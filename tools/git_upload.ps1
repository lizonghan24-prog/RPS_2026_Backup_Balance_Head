$ErrorActionPreference = "Stop"

[Console]::InputEncoding = [System.Text.Encoding]::UTF8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

function Write-Step {
    param([string]$Message)
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Invoke-Git {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)

    & git @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "git $($Arguments -join ' ') failed, exit code: $LASTEXITCODE"
    }
}

try {
    Write-Step "Check Git repository"
    $repoRoot = (& git rev-parse --show-toplevel).Trim()
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($repoRoot)) {
        throw "Current directory is not a Git repository."
    }
    Set-Location $repoRoot

    $branch = (& git branch --show-current).Trim()
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($branch)) {
        throw "Current HEAD is not on a normal branch, cannot auto push."
    }

    $remoteNames = @(& git remote)
    foreach ($remoteName in @("origin", "balance")) {
        if ($remoteNames -notcontains $remoteName) {
            throw "Missing remote: $remoteName"
        }
    }

    Write-Host "Repo: $repoRoot"
    Write-Host "Branch: $branch"
    Write-Host "Targets: origin/$branch and balance/$branch"
    Write-Host ""
    Write-Host "Note: this tool uses normal git add -A -- ., so .gitignore is respected." -ForegroundColor Yellow

    $title = Read-Host "Commit title"
    if ([string]::IsNullOrWhiteSpace($title)) {
        Write-Host "Empty commit title, cancelled."
        exit 0
    }

    Write-Step "Stage changes with .gitignore respected"
    Invoke-Git @("add", "-A", "--", ".")

    Write-Step "Changes to commit"
    Invoke-Git @("status", "--short")

    & git diff --cached --quiet --exit-code
    if ($LASTEXITCODE -eq 0) {
        Write-Host ""
        Write-Host "No changes to commit."
        exit 0
    }

    Write-Step "Create commit"
    Invoke-Git @("commit", "-m", $title)

    Write-Step "Push to origin"
    Invoke-Git @("push", "origin", $branch)

    Write-Step "Push to balance"
    Invoke-Git @("push", "balance", $branch)

    Write-Host ""
    Write-Host "Done: committed and pushed to both repositories." -ForegroundColor Green
}
catch {
    Write-Host ""
    Write-Host "Failed: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "Please check the Git output above, then retry."
    exit 1
}
