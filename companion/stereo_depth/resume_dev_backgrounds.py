from pathlib import Path
import os,signal,json
p=Path(__file__).resolve().parent/'optimization_20260914/paused_backgrounds.json'
for item in json.loads(p.read_text()):
 proc=Path('/proc')/str(item['pid'])
 try:
  if proc.joinpath('stat').read_text().split()[21]==item['starttime'] and item['needle'] in proc.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode():
   os.kill(item['pid'],signal.SIGCONT);print('Resumed',item['pid'])
 except (FileNotFoundError,ProcessLookupError):pass
