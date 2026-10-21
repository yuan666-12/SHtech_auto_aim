#!/usr/bin/env python3
"""Report project functions longer than 100 physical lines; requires lizard."""
import argparse
import ast
from pathlib import Path


def main():
    import lizard
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-lines', type=int, default=100)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    excluded = {'.git', 'thirdparty', 'build', 'build_real', 'build_sim', 'build_checks',
                'log', 'logs', 'frames', 'calib_frames'}
    sdk = {'CameraParams.h', 'MvCameraControl.h', 'MvErrorDefine.h', 'PixelType.h'}
    violations = []
    count = 0
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root)
        if not path.is_file() or relative.parts[0] in excluded or path.name in sdk:
            continue
        if {'tinympc', '__pycache__'}.intersection(relative.parts):
            continue
        if path.suffix not in {'.cpp', '.hpp', '.h', '.c', '.py'}:
            continue
        source = path.read_text(encoding='utf-8-sig')
        if path.suffix == '.py':
            functions = [(node.name, node.lineno, node.end_lineno) for node in ast.walk(ast.parse(source))
                         if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]
        else:
            functions = [(f.name, f.start_line, f.end_line) for f in
                         lizard.analyze_file.analyze_source_code(str(path), source).function_list]
        count += len(functions)
        violations.extend((str(relative), name, start, end-start+1)
                          for name, start, end in functions if end-start+1 > args.max_lines)
    for path, name, start, length in violations:
        print(f'{path}:{start}: {name}: {length} lines')
    print(f'Checked {count} functions; {len(violations)} exceed {args.max_lines} lines')
    return bool(violations)


if __name__ == '__main__':
    raise SystemExit(main())
