import sys
import os
from typing import List, Tuple
import pkgutil
from . import scripts, cwipc_get_version

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
    print("\nSpecial commands:", file=sys.stderr)
    print("  help                 - show this help message", file=sys.stderr)
    print("  version              - show cwipc version", file=sys.stderr)
    print("  check                - check if cwipc is correctly installed, or fix installation", file=sys.stderr)
    print("  python               - run python that has cwipc package installed", file=sys.stderr)
    print("\nUse 'cwipc <command> -h' for help on a specific command.", file=sys.stderr)
    print("See http://github.com/cwi-dis/cwipc for more information.", file=sys.stderr)

def run_version() -> int:
    print(cwipc_get_version())
    return 0

def run_check() -> int:
    mydir = os.path.dirname(__file__)
    while True:
        totry = [
            os.path.join(mydir, 'bin', 'cwipc_check.exe'),
            os.path.join(mydir, 'bin', 'cwipc_check'),
            os.path.join(mydir, 'build', 'bin', 'cwipc_check'),
            os.path.join(mydir, 'build', 'bin', 'RelWithDebInfo', 'cwipc_check.exe'),
            os.path.join(mydir, 'build', 'bin', 'Release', 'cwipc_check.exe'),
        ]
        for path in totry:
            if os.path.exists(path):
                check_script = path
                os.execv(check_script, [check_script] + sys.argv[2:])
                assert 0, "execv failed"
        parent = os.path.dirname(mydir)
        if parent == mydir:
            print("Could not find cwipc_check executable", file=sys.stderr)
            return 1
        mydir = parent
    
def run_python() -> None:
    os.execv(sys.executable, [sys.executable] + sys.argv[2:])
    assert 0, "execv failed"

def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help', 'help'):
        help()
        sys.exit(1)
    command = sys.argv[1]
    if command in ('-v', '--version', 'version'):
        sys.exit(run_version())
    if command == "check":
        sys.exit(run_check())
    if command == "python":
        sys.exit(run_python())
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