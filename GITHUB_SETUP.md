# Setting up GitHub Repository

Your local repository is ready! All files have been committed. Now you need to create a GitHub repository and push your code.

## Option 1: Using GitHub Website (Recommended for beginners)

### Step 1: Create Repository on GitHub

1. Go to https://github.com/new
2. Fill in the repository details:
   - **Repository name**: `fluxspace-2d-testbed` (or any name you prefer)
   - **Description**: "2D drone testbed simulator for FluxSpace - testing path planning and autonomy"
   - **Visibility**: Choose Public or Private
   - **DO NOT** initialize with README, .gitignore, or license (we already have these)
3. Click "Create repository"

### Step 2: Connect and Push

After creating the repository, GitHub will show you commands. Use these commands in your terminal:

```bash
# Add the remote repository (replace YOUR_USERNAME with your GitHub username)
git remote add origin https://github.com/YOUR_USERNAME/fluxspace-2d-testbed.git

# Rename branch to main (if needed)
git branch -M main

# Push your code
git push -u origin main
```

If you're using SSH instead of HTTPS:
```bash
git remote add origin git@github.com:YOUR_USERNAME/fluxspace-2d-testbed.git
git branch -M main
git push -u origin main
```

## Option 2: Using GitHub CLI (if installed)

If you have GitHub CLI installed, you can create the repository from the command line:

```bash
# Create repository and push in one command
gh repo create fluxspace-2d-testbed --public --source=. --remote=origin --push
```

Or for a private repository:
```bash
gh repo create fluxspace-2d-testbed --private --source=. --remote=origin --push
```

## After Pushing

Once pushed, you can:
- View your code at: `https://github.com/YOUR_USERNAME/fluxspace-2d-testbed`
- Share the repository with others
- Continue making changes and pushing updates with:
  ```bash
  git add .
  git commit -m "Your commit message"
  git push
  ```

## Troubleshooting

### Authentication Issues

If you get authentication errors:
- **HTTPS**: GitHub no longer accepts passwords. Use a Personal Access Token:
  1. Go to GitHub Settings → Developer settings → Personal access tokens → Tokens (classic)
  2. Generate a new token with `repo` scope
  3. Use the token as your password when prompted

- **SSH**: Set up SSH keys:
  1. Generate SSH key: `ssh-keygen -t ed25519 -C "your_email@example.com"`
  2. Add to GitHub: Settings → SSH and GPG keys → New SSH key
  3. Test: `ssh -T git@github.com`

### Branch Name Issues

If you see warnings about branch names:
```bash
# Rename to main
git branch -M main
git push -u origin main
```

