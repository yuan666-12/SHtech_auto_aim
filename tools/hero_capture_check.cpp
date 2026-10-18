#include "common.hpp"
#include "grab_strategy.hpp"
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;
int checks = 0;
void require(bool ok, const char* text) {
    ++checks;
    if (!ok) throw std::runtime_error(text);
}

// SDK policy test is a recording stub. No camera handles are opened.
int sdk_calls = 0, sdk_strategy = -1, sdk_result = 0;
void* sdk_handle = nullptr;
extern "C" int MV_CC_SetGrabStrategy(void* handle, int strategy) {
    ++sdk_calls; sdk_handle = handle; sdk_strategy = strategy;
    return sdk_result;
}

struct FakeSensor : pipeline::SubModule {
    std::atomic<int>& captures;
    explicit FakeSensor(std::atomic<int>& count)
        : SubModule(SubModuleName::SENSOR), captures(count) {}
    SubModuleResult process(std::shared_ptr<ThreadDataPack> data,
                            const pipeline::BasicTask*) override {
        data->index = ++captures;
        data->time = std::chrono::high_resolution_clock::now();
        return SubModuleResult::SUCCESS;
    }
};

void capture_order_checks() {
    pipeline::PipelineTask producer;
    pipeline::BasicTask reader;
    pipeline::AutoAimQueue pool(2);
    pipeline::AutoAimHandshake images(0);
    for (int i = 0; i != 2; ++i) pool.put(std::make_shared<ThreadDataPack>());
    std::atomic<int> captures{0};
    if (!producer.register_submodule_with_params<FakeSensor>(captures))
        throw std::runtime_error("could not register capture test module");
    std::mutex mutex;
    std::condition_variable changed;
    bool allow_first = false, allow_next = false;
    struct Sample {
        int index;
        std::chrono::high_resolution_clock::time_point captured_at;
        double ready_wait_ms;
    };
    std::vector<Sample> samples;
    producer.start(); reader.start();
    std::thread producing([&] { producer(pool, images); });
    std::thread consuming([&] {
        {
            std::unique_lock<std::mutex> lock(mutex);
            changed.wait(lock, [&] { return allow_first || !reader.isalive(); });
        }
        for (int i = 0; i != 3 && reader.isalive(); ++i) {
            auto data = images.get(&reader);
            if (!data) return;
            {
                std::lock_guard<std::mutex> lock(mutex);
                samples.push_back({data->index, data->time, data->capture_ready_wait_ms});
            }
            changed.notify_all();
            pool.put(std::move(data), &reader);
            if (i == 0) {
                std::unique_lock<std::mutex> lock(mutex);
                changed.wait(lock, [&] { return allow_next || !reader.isalive(); });
            }
        }
    });
    struct Cleanup {
        pipeline::BasicTask& producer;
        pipeline::BasicTask& reader;
        std::mutex& mutex;
        std::condition_variable& changed;
        std::thread& producing;
        std::thread& consuming;
        ~Cleanup() {
            {
                std::lock_guard<std::mutex> lock(mutex);
                producer.terminate(); reader.terminate();
            }
            changed.notify_all();
            if (producing.joinable()) producing.join();
            if (consuming.joinable()) consuming.join();
        }
    } cleanup{producer, reader, mutex, changed, producing, consuming};

    std::this_thread::sleep_for(25ms);
    require(captures == 0, "capture cannot begin before detector requests a frame");
    {
        std::lock_guard<std::mutex> lock(mutex); allow_first = true;
    }
    changed.notify_all();
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, 1s, [&] { return !samples.empty(); }),
                "first consumer request starts acquisition");
    }
    std::this_thread::sleep_for(30ms);
    require(captures == 1, "no next-frame capture while detector processes the previous frame");
    const auto released_at = std::chrono::high_resolution_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex); allow_next = true;
    }
    changed.notify_all();
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(changed.wait_for(lock, 1s, [&] { return samples.size() == 3; }),
                "multiple capture/detect handoffs complete without deadlock");
        require(samples[1].captured_at >= released_at,
                "second image is acquired after detector becomes ready, not before");
        require(samples[1].ready_wait_ms >= 20,
                "consumer wait is measured before acquisition");
        require(samples[0].index == 1 && samples[1].index == 2 && samples[2].index == 3,
                "frames are handed off once in acquisition order");
    }
    const auto stopped_at = std::chrono::steady_clock::now();
    producer.terminate(); producing.join();
    require(std::chrono::steady_clock::now() - stopped_at < 300ms,
            "stopping while waiting for detector does not hang producer");
}

void cancellation_checks() {
    pipeline::HandshakePipeline<int> pipe(0);
    pipeline::BasicTask reader, producer;
    reader.start(); producer.start();
    std::shared_ptr<int> result;
    std::thread consuming([&] { result = pipe.get(&reader); });
    std::this_thread::sleep_for(25ms); reader.terminate(); consuming.join();
    require(!result, "cancelled consumer returns without a frame");
    std::atomic<bool> returned{false}; bool ready = true;
    std::thread waiting([&] { ready = pipe.wait_for_consumer(&producer); returned = true; });
    std::this_thread::sleep_for(25ms);
    const bool was_waiting = !returned;
    producer.terminate(); waiting.join();
    require(was_waiting && !ready, "cancelled consumer cannot leave stale acquisition permission");

    pipeline::HandshakePipeline<int> legacy(0);
    std::thread sending([&] { legacy.put(std::make_shared<int>(7)); });
    auto value = legacy.get(); sending.join();
    require(value && *value == 7, "existing handshake put/get remains compatible");
    pipeline::BufferedPipeline<int> buffered(2);
    buffered.put(std::make_shared<int>(9));
    require(*buffered.get() == 9, "buffered queues retain their existing behavior");
}

int main() {
    try {
        auto* handle = reinterpret_cast<void*>(0x1234);
        require(hikcam::select_latest_frame(handle) == 0 && sdk_calls == 1 &&
                sdk_handle == handle && sdk_strategy == 1,
                "camera configuration requests documented LatestImagesOnly policy");
        sdk_result = -1;
        require(hikcam::select_latest_frame(handle) == -1,
                "SDK rejection is propagated instead of claiming latest-frame acquisition");
        capture_order_checks(); cancellation_checks();
        std::cout << "PASS " << checks << " capture freshness and lifecycle checks; no camera/UART opened\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
