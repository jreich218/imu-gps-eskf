# Agents

This app does not do any data verification. To see why, please read `/home/dfw/dev/quarto_site/eskf_docs/inputs/index.qmd` now. The short summary is that **everything** we would verify on the input data is already known to hold true for the input data this app allows. And having fake data verification for performance is not permitted.

Any time I mention "project docs", I am talking about the content in the `eskf_docs/` folder at `/home/dfw/dev/quarto_site/eskf_docs`. Do not read the `eskf_docs/` folder in this repo, it is a stale mirror of the Quarto-site docs.

Please read the project docs at `/home/dfw/dev/quarto_site/eskf_docs`, the `README.md` at the root of this repo, and the code in this repo in order to orient yourself. Please also read the older version of this app at `/home/dfw/dev/imu-gps-eskf-version-being-rewritten`. There are important things to note about the old version of this app and how it relates to this version:

1. This version *will* have the same features the old one had (up to details that the current Quarto docs conflict with in which case we go with the current Quarto docs).
2. This version *will* be an ESKF implementation that works on the nuScenes and bundled simulated-input-data, just like the old version. Both the old version and this version use simulated GPS data and IMU data to run an ESKF. **THIS IS THE NON-NEGOTIABLE THING THAT THE OLD VERSION AND THE NEW VERSION WILL ALWAYS BE 100% IN AGREEMENT ABOUT**.
3. This version will *not* use the exact same design (although it might in places, and it is certainly welcome to).
4. This version will *not* necessarily do every single internal detail in the same way the old version does (although this version is free to do things the same way).
5. This version will *not* necessarily use the exact same names the old version uses (although this version is free to name things the same way).
6. Wherever the older app includes a test for an impossible input shape (e.g., where two or more pose samples occur after the last IMU sample) we will **NOT** port that test or add an equivalent here. This is because the allowed input files for this app are permanently fixed; in this example, the Quarto docs document that pose files never contain two or more pose samples after the last IMU sample. So spending code or tests on it breaks the project rules. Another example: If the old version rejects nonpositive `dt` in its predict implementation we will **NOT** do the same here. Same reason: The input data is already guaranteed to not have such a case.

**GOLDEN RULE**: Because of #6 from the above list, the golder rule for comparing our project to the old version is this: We override the old version when it is in conflict with either the current Quarto documentation and/or rules+context I've provided with this chat! We also, as #3 through #5 suggest from the above list use our own design, names, and implementation details when we wish to improve upon the old version.

Regardless of what my first prompt happens to be, please briefly include in your first answer a summary of both the reading you did and your understanding of what I discussed above.
