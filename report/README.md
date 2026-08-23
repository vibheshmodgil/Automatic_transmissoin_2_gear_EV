# Generated report

`shift_study_report.html` is a self-contained HTML report with every figure embedded
as a data URI — no external files, so it can be emailed or opened offline.

Regenerate it after a run with:

```bash
python ../make_shift_report.py               # -> shift_study_report.html
python ../make_shift_report.py --body-only   # -> a fragment, for embedding
```

The copy checked in here was generated from the **synthetic** sample data. Regenerate
it against the real drive cycle and dyno map before circulating it.
