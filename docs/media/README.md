# docs/media

Images and GIFs used by the README and devlog entries.

## Rules

- **Only small images and GIFs.** Every file must be **< 5 MB**. Compress before committing.
- **No videos in the repo.** Upload videos elsewhere (e.g. YouTube, LinkedIn) and **link** them from the README or devlog entry. Video extensions are blocked by `.gitignore`.
- **Naming:** `YYYY-MM-DD-short-description.ext`, e.g. `2026-10-25-first-motor-test.gif`.
- Prefer `.png` for screenshots and diagrams, `.jpg` for photos, `.gif` (or `.webp`) for short loops.

Check sizes before committing:

```bash
find docs/media -type f -size +5M
```
