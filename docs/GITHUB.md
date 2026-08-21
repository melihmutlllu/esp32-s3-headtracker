# Publishing via the GitHub Website

This project is meant to be uploaded through github.com's web UI, not `git push`.
No local git setup is required.

## Before Uploading

**Do not upload `firmware/headtracker_udp_lowpower/secrets.h` if you created one.**
`.gitignore` only protects you when you use `git`; a browser upload has no such
filter, so it's on you to leave that file out manually. Only
`secrets.example.h` should go up.

## Steps

1. On GitHub, create a new repository (e.g. `esp32-s3-headtracker`). Leave
   "Add a README" / `.gitignore` / license unchecked — this project already
   has them.
2. Open the empty repo, click **Add file > Upload files**.
3. Drag the whole project folder in (GitHub preserves subfolder structure —
   `firmware/`, `hardware/`, `docs/`, `tools/` will show up as-is). If it
   only accepts loose files in your browser, upload folder by folder instead.
4. Double-check `secrets.h` isn't in the list of staged files before you commit.
5. Commit directly to `main`.

## Limits to Know

- 25 MiB per file via the web uploader, up to 100 files per upload batch.
  The largest file here is `hardware/altium/PCB1.PcbDoc` at ~3.8 MB, so
  everything fits in one pass.
- If GitHub ever flags a file as too large or the folder drag doesn't work
  in your browser, upload `hardware/altium/` separately from the rest.

## Altium Sources

Already included in `hardware/altium/`:

```text
headtracking_.PrjPcb   Project file
headtracking.SchDoc    Schematic
PCB1.PcbDoc             Board layout
Schlib1.SchLib          Custom schematic symbols
PcbLib2.PcbLib          Custom PCB footprints
```

If you regenerate these in Altium later and re-upload, skip committing its
generated/local output folders (`History/`, `__Previews/`, `Project Outputs*/`).
