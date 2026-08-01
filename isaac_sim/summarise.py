import re, sys
def parse(path):
    trials=[]; cur=None
    for line in open(path):
        m=re.match(r'##### trial (\d+) (\S+)', line)
        if m: cur={'lap':f"{m.group(1)} {m.group(2)}"}; trials.append(cur); continue
        if cur is None: continue
        if line.startswith('result:'): cur['result']=line.split(':',1)[1].strip()
        m=re.search(r'samples inside the [\d.]+ m corridor: (\d+)', line)
        if m: cur['inside']=int(m.group(1))
        m=re.search(r'x range in corridor: ([\d.]+) \.\. ([\d.]+)', line)
        if m: cur['xmin'],cur['xmax']=float(m.group(1)),float(m.group(2))
    return trials
for path in sys.argv[1:]:
    ts=parse(path)
    print(f"\n=== {path.split('/')[-1]} ===")
    att=ok=0
    for t in ts:
        transit = t.get('inside',0)>0
        r=t.get('result','?')
        # a lap that never reached the corridor is not an attempt
        if not transit and r=='SUCCEEDED':
            note='(no transit needed)'
        else:
            att+=1
            if r=='SUCCEEDED' and transit: ok+=1
            note=''
        cl = f"clr {min(t['xmin']-6.023, 6.956-t['xmax'])*100-29:.1f} cm" if 'xmin' in t else ''
        print(f"  lap {t['lap']:<8} {r:<12} inside={t.get('inside',0):<4} {cl} {note}")
    print(f"  -> corridor transits attempted {att}, passed {ok} ({100*ok/att if att else 0:.0f}%)")
