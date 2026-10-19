# Publish an existing folder to GitHub

Run these inside the Linux machine, from `~/super_sim` (or wherever you moved it).
This project is not initialized automatically by the cleanup.

1. Review `.gitignore` and the files you intend to publish. Keep generated outputs,
   credentials and large recordings out of Git. Confirm redistribution rights for
   models/assets; the package's license field alone does not license every asset.
2. Create an empty repository named `super_sim` on GitHub. Choose its visibility.
   Do not initialize the remote with a README, license or `.gitignore`.
3. If this directory is not already a Git repository:

```bash
git init -b main
git add .
git status --short
git diff --cached --stat
git commit -m "Initial simulator source"
```

4. Connect the empty GitHub repository and push:

```bash
git remote add origin https://github.com/DemirKucukdemiral/super_sim.git
git remote -v
git push -u origin main
```

If Git is already initialized, keep its history and skip initialization. If an
`origin` exists, inspect it before changing anything. Authenticate using GitHub's
supported credential flow or SSH; do not use your account password for HTTPS Git.

Source: https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github
