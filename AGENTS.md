# Agents

This app does not do any data verification. Because of the first paragraph of `/home/dfw/dev/quarto_site/docs/theory/supported-input/index.qmd`, and the fact that this app only allows inputs from the files discussed on that page, having fake data verification for performance is not permitted.

Any time I mention "project docs", I am talking about the content in the `docs/` folder at `/home/dfw/dev/quarto_site/docs`. Do not read the `docs/` folder in this repo, it is a stale mirror of the Quarto-site docs.

Please read the project docs at `/home/dfw/dev/quarto_site/docs`, the `README.md` at the root of this repo, and the code in this repo in order to orient yourself. Please also read the older version of this app at `/home/dfw/dev/state-estimation-version-being-rewritten`. There are important things to note about the old version of this app and how it relates to this version:

1. This version *will* have the same features the old one had.
2. This version *will* be an ESKF implementation that works on the nuScenes and bundled simulated-input-data, just like the old version. The filter uses simulated GPS data and IMU data.
3. This version will *not* use the exact same design (although it might in places, and it is certainly welcome to).
4. This version will *not* necessarily do every single internal detail in the same way the old version does (although this version is free to do things the same way).
5. This version will *not* necessarily use the exact same names the old version uses (although this version is free to name things the same way).

Regardless of what my first prompt happens to be, please briefly include in your first answer a summary of both the reading you did and your understanding of what I discussed above.
