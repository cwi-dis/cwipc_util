import sys
from typing import List, Tuple
import pkgutil
from . import scripts

MAIN_COMMANDS = ['view', 'play', 'grab', 'register']

def find_scripts() -> Tuple[List[str], List[str]]:
    scripts_list = []
    for _, name, _ in pkgutil.iter_modules(scripts.__path__):
        if not name.startswith('cwipc_'):
            continue
        command = name[6:]
        if command in MAIN_COMMANDS:
            continue
        scripts_list.append(command)
    return MAIN_COMMANDS, scripts_list

def get_docstring(name : str) -> str:
    mod = __import__(f'cwipc.scripts.cwipc_{name}', fromlist=[''])
    if mod.__doc__ is None:
        return ""
    return mod.__doc__.strip().split('\n')[0]
    
def help():
    main_scripts, auxiliary_scripts = find_scripts()
    print(f"{sys.argv[0]} - CWI Point Cloud command line utility", file=sys.stderr)
    print("\nMain commands:", file=sys.stderr)
    for s in main_scripts:
        doc = get_docstring(s)
        if doc:
            print(f"  {s:20} - {doc}", file=sys.stderr)
        else:
            print(f"  {s:20}", file=sys.stderr)
    print("\nAdditional commands:", file=sys.stderr)
    for s in auxiliary_scripts:
        doc = get_docstring(s)
        if doc:
            print(f"  {s:20} - {doc}", file=sys.stderr)
        else:
            print(f"  {s:20}", file=sys.stderr)
    print("\nUse 'cwipc <command> -h' for help on a specific command.", file=sys.stderr)
    print("See http://github.com/cwi-dis/cwipc for more information.", file=sys.stderr)

def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help', 'help'):
        help()
        sys.exit(1)
    command = sys.argv[1]
    try:
        mod = __import__(f'cwipc.scripts.cwipc_{command}', fromlist=[''])
    except ImportError:
        print(f"Unknown command '{command}'. Use -h for help.")
        sys.exit(1)
    sys.argv[0] = sys.argv[0] + ' ' + command
    del sys.argv[1]
    sys.exit(mod.main())
    
if __name__ == '__main__':
    main()