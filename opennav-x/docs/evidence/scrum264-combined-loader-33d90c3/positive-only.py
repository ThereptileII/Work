from pathlib import Path
import sys
p=Path(__file__).resolve().parents[2]/'tools/verify-anchor-loader.py'
s=p.read_text()
marker="    if args.seamarks:\n        original=(output/'light-render-methods.inc').read_text()\n"
assert s.count(marker)==1
s=s.split(marker)[0]+"\nif __name__ == '__main__':\n    main()\n"
exec(compile(s,str(p),'exec'),{'__file__':str(p),'__name__':'__main__'})
