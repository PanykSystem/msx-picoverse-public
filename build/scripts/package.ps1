param(
    [switch]$Clean,
    [switch]$Commit,
    [switch]$Release,
    [string]$Repository
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-ToolDescription {
    param(
        [string]$VersionDir,
        [string]$FileName
    )

    $name = [System.IO.Path]::GetFileNameWithoutExtension($FileName).ToLowerInvariant()

    switch ("$VersionDir/$name") {
        "2040/loadrom" {
            return "Creates UF2 images for PicoVerse 2040 single-ROM boots, Sunrise IDE Nextor USB mode, Sunrise IDE plus 192KB mapper mode, or standalone USB keyboard, MSX-MIDI, MIDI-PAC, and USB joystick firmware."
        }
        "2040/multirom" {
            return "Builds a PicoVerse 2040 MultiROM UF2 that bundles multiple MSX ROMs into one menu-driven image, with optional Sunrise IDE Nextor USB and Nextor plus 192KB mapper SYSTEM entries."
        }
        "2350/loadrom" {
            return "Creates UF2 images for PicoVerse 2350 single-ROM boots, Sunrise IDE Nextor from microSD or USB, optional 256KB mapper modes, and SCC or SCC+ flags for compatible Konami SCC and Manbow2 ROMs."
        }
        "2350/multirom" {
            return "Builds a PicoVerse 2350 MultiROM UF2 with multiple MSX ROMs, optional Nextor microSD and USB SYSTEM entries, optional 256KB mapper modes, and SCC or SCC+ audio emulation for compatible ROMs."
        }
        "2350/explorer" {
            return "Builds the PicoVerse 2350 Explorer UF2, combining flash ROM packaging with microSD folder browsing, on-device search, 40/80-column menu support, and MP3 playback on the MSX."
        }
        "2350/yamanooto" {
            return "Creates UF2 images for PicoVerse 2350 Yamanooto flash-cartridge emulation, bundling a ROM image with SCC/SCC+, dual PSG, PSG mirror, and MSX-MUSIC/FM-PAC support."
        }
        default {
            return "Windows tool included in this release package."
        }
    }
}

function Get-ToolDocPath {
    param(
        [string]$VersionDir,
        [string]$FileName
    )

    $name = [System.IO.Path]::GetFileNameWithoutExtension($FileName).ToLowerInvariant()

    switch ("$VersionDir/$name") {
        "2040/loadrom" {
            return "docs/msx-picoverse-2040-loadrom-tool-manual.en-us.md"
        }
        "2040/multirom" {
            return "docs/msx-picoverse-2040-multirom-tool-manual.en-us.md"
        }
        "2350/loadrom" {
            return "docs/msx-picoverse-2350-loadrom-tool-manual.en-us.md"
        }
        "2350/multirom" {
            return "docs/msx-picoverse-2350-multirom-tool-manual.en-us.md"
        }
        "2350/explorer" {
            return "docs/msx-picoverse-2350-explorer-tool-manual.en-us.md"
        }
        "2350/yamanooto" {
            return "docs/msx-picoverse-2350-yamanooto-tool-manual.en-us.md"
        }
        default {
            return $null
        }
    }
}

function Get-ReleaseAssetLabel {
    param(
        [string]$FilePath
    )

    $extension = [System.IO.Path]::GetExtension($FilePath).ToLowerInvariant()
    switch ($extension) {
        ".zip" { return "Release package" }
        ".txt" { return "Release notes" }
        default { return [System.IO.Path]::GetFileName($FilePath) }
    }
}

function Get-RelativePath {
    param(
        [string]$BasePath,
        [string]$Path
    )

    $baseUri = [System.Uri]::new(($BasePath.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar))
    $pathUri = [System.Uri]::new($Path)
    return [System.Uri]::UnescapeDataString($baseUri.MakeRelativeUri($pathUri).ToString()).Replace('/', [System.IO.Path]::DirectorySeparatorChar)
}

function Get-ExecutableVersion {
    param(
        [string]$ExePath
    )

    if (-not (Test-Path -LiteralPath $ExePath -PathType Leaf)) {
        throw "Required UF2 tool was not found: $ExePath"
    }

    try {
        $output = (& $ExePath --help 2>&1 | Out-String)
    } catch {
        throw "Could not run UF2 tool to read its version: $ExePath`n$($_.Exception.Message)"
    }
    if ($output -match 'v\d+(?:\.\d+)+') {
        return $Matches[0]
    }

    $fileVersion = [System.Diagnostics.FileVersionInfo]::GetVersionInfo($ExePath).ProductVersion
    if ($fileVersion) {
        return $fileVersion
    }

    return "unknown"
}

function Get-VersionedUf2Path {
    param(
        [string]$OutputPath,
        [string]$Version
    )

    $directory = Split-Path -Parent $OutputPath
    $name = [System.IO.Path]::GetFileNameWithoutExtension($OutputPath)
    $extension = [System.IO.Path]::GetExtension($OutputPath)
    $safeVersion = ($Version -replace '[^A-Za-z0-9._-]', '_')
    return (Join-Path $directory ("$name`_$safeVersion$extension"))
}

function Remove-OldVersionedUf2Files {
    param(
        [string]$OutputPath,
        [string]$CurrentVersionedOutputPath
    )

    $directory = Split-Path -Parent $OutputPath
    if (-not (Test-Path -LiteralPath $directory -PathType Container)) {
        return
    }

    $name = [System.IO.Path]::GetFileNameWithoutExtension($OutputPath)
    $extension = [System.IO.Path]::GetExtension($OutputPath)
    $pattern = "$name`_v*$extension"

    Get-ChildItem -LiteralPath $directory -File -Filter $pattern |
        Where-Object { $_.FullName -ne $CurrentVersionedOutputPath } |
        Remove-Item -Force
}

function Remove-ObsoleteFirmwareUf2Files {
    param(
        [string]$Root,
        [System.Collections.Generic.List[object]]$FirmwareEntries
    )

    $keepPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($entry in $FirmwareEntries) {
        [void]$keepPaths.Add((Join-Path $Root $entry.VersionedFile))
    }

    foreach ($firmwareDir in @((Join-Path $Root "firmware\2040"), (Join-Path $Root "firmware\2350"))) {
        if (-not (Test-Path -LiteralPath $firmwareDir -PathType Container)) {
            continue
        }

        Get-ChildItem -LiteralPath $firmwareDir -File -Filter *.uf2 |
            Where-Object { -not $keepPaths.Contains($_.FullName) } |
            Remove-Item -Force
    }
}

function Invoke-Uf2Tool {
    param(
        [string]$Root,
        [string]$ExePath,
        [string[]]$Options,
        [string]$OutputPath,
        [string]$WorkingDirectory
    )

    $version = Get-ExecutableVersion -ExePath $ExePath
    $versionedOutputPath = Get-VersionedUf2Path -OutputPath $OutputPath -Version $version
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OutputPath) | Out-Null
    Remove-OldVersionedUf2Files -OutputPath $OutputPath -CurrentVersionedOutputPath $versionedOutputPath

    $localOutputName = [System.IO.Path]::GetFileName($versionedOutputPath)

    if (Test-Path -LiteralPath $OutputPath) {
        Remove-Item -Force -LiteralPath $OutputPath
    }
    if (Test-Path -LiteralPath $versionedOutputPath) {
        Remove-Item -Force -LiteralPath $versionedOutputPath
    }

    $startDirectory = (Get-Location).Path
    try {
        if ($WorkingDirectory) {
            if (-not (Test-Path -LiteralPath $WorkingDirectory -PathType Container)) {
                throw "Required UF2 working directory was not found: $WorkingDirectory"
            }
            Set-Location -LiteralPath $WorkingDirectory
        }

        if (Test-Path -LiteralPath $localOutputName) {
            Remove-Item -Force -LiteralPath $localOutputName
        }

        $arguments = @($Options) + @("-o", $localOutputName)
        try {
            $toolOutput = (& $ExePath @arguments 2>&1 | Out-String)
        } catch {
            throw "Could not run UF2 tool: $ExePath $($arguments -join ' ')`n$($_.Exception.Message)"
        }
        if ($LASTEXITCODE -ne 0) {
            throw "UF2 generation failed with exit code $LASTEXITCODE for $ExePath $($arguments -join ' ')`n$toolOutput"
        }

        if (-not (Test-Path -LiteralPath $localOutputName -PathType Leaf)) {
            throw "UF2 tool did not create the expected output: $localOutputName"
        }

        Move-Item -Force -LiteralPath $localOutputName -Destination $versionedOutputPath
    } finally {
        Set-Location -LiteralPath $startDirectory
    }

    if (-not (Test-Path -LiteralPath $versionedOutputPath -PathType Leaf)) {
        throw "UF2 tool did not create the expected output: $versionedOutputPath"
    }

    return [pscustomobject]@{
        Hardware       = if ($OutputPath -match '[\\/]2040[\\/]') { "2040" } else { "2350" }
        Tool           = [System.IO.Path]::GetFileName($ExePath)
        ToolVersion    = $version
        VersionedFile  = Get-RelativePath -BasePath $Root -Path $versionedOutputPath
        Options        = ($Options -join ' ')
    }
}

function New-FirmwareUf2Files {
    param(
        [string]$Root
    )

    $loadrom2040 = Join-Path $Root "2040\software\loadrom.pio\tool\dist\loadrom.exe"
    $multirom2040 = Join-Path $Root "2040\software\multirom.pio\tool\dist\multirom.exe"
    $loadrom2350 = Join-Path $Root "2350\software\loadrom.pio\tool\dist\loadrom.exe"
    $multirom2350 = Join-Path $Root "2350\software\multirom.pio\tool\dist\multirom.exe"
    $explorer2350 = Join-Path $Root "2350\software\explorer.pio\tool\dist\explorer.exe"
    $yamanooto2350 = Join-Path $Root "2350\software\yamanooto\tool\dist\yamanooto.exe"
    $konami2040 = Join-Path $Root "2040\software\multirom.pio\tool\build\konami"
    $konami2350 = Join-Path $Root "2350\software\multirom.pio\tool\build\konami"
    $yamanootoBuild = Join-Path $Root "2350\software\yamanooto\tool\build"

    $specs = @(
        @{ Exe = $loadrom2040; Options = @("-j"); Output = "firmware\2040\loadrom_joystick.uf2" },
        @{ Exe = $loadrom2040; Options = @("-k"); Output = "firmware\2040\loadrom_keyboard.uf2" },
        @{ Exe = $loadrom2040; Options = @("-i"); Output = "firmware\2040\loadrom_midi.uf2" },
        @{ Exe = $loadrom2040; Options = @("-p"); Output = "firmware\2040\loadrom_midipac.uf2" },
        @{ Exe = $loadrom2040; Options = @("-m"); Output = "firmware\2040\loadrom_nextor_192kb.uf2" },
        @{ Exe = $loadrom2040; Options = @("-s"); Output = "firmware\2040\loadrom_nextor.uf2" },
        @{ Exe = $multirom2040; Options = @("-m"); Output = "firmware\2040\multirom_konami_msx1_roms_plus_nextor_192kb.uf2"; WorkingDirectory = $konami2040 },
        @{ Exe = $multirom2040; Options = @("-s"); Output = "firmware\2040\multirom_konami_msx1_roms_plus_nextor.uf2"; WorkingDirectory = $konami2040 },
        @{ Exe = $multirom2040; Options = @(); Output = "firmware\2040\multirom_konami_msx1_roms.uf2"; WorkingDirectory = $konami2040 },
        @{ Exe = $loadrom2350; Options = @("-m1", "-w"); Output = "firmware\2350\loadrom_nextor_1mb_sd_wifi.uf2" },
        @{ Exe = $loadrom2350; Options = @("-m2", "-w"); Output = "firmware\2350\loadrom_nextor_1mb_usb_wifi.uf2" },
        @{ Exe = $loadrom2350; Options = @("-r1"); Output = "firmware\2350\loadrom_nextor_1mb_sd_megaram.uf2" },
        @{ Exe = $loadrom2350; Options = @("-r2"); Output = "firmware\2350\loadrom_nextor_1mb_usb_megaram.uf2" },
        @{ Exe = $loadrom2350; Options = @("-4", "--opl4-limit"); Output = "firmware\2350\loadrom_opl4_adaptative_limiter.uf2" },
        @{ Exe = $loadrom2350; Options = @("-4", "--opl4-limit", "--lowclock"); Output = "firmware\2350\loadrom_opl4_adaptative_limiter_lowclock.uf2" },
        @{ Exe = $loadrom2350; Options = @("-4", "--lowclock"); Output = "firmware\2350\loadrom_opl4_lowclock.uf2" },
        @{ Exe = $loadrom2350; Options = @("-4"); Output = "firmware\2350\loadrom_opl4.uf2" },
        @{ Exe = $multirom2350; Options = @("-s1", "-m1", "-s2", "-m2", "-c1", "-c2"); Output = "firmware\2350\multirom_all_nextor.uf2" },
        @{ Exe = $multirom2350; Options = @("-m1"); Output = "firmware\2350\multirom_konami_msx1_roms_plus_nextor_sd_1mb.uf2"; WorkingDirectory = $konami2350 },
        @{ Exe = $multirom2350; Options = @("-m2"); Output = "firmware\2350\multirom_konami_msx1_roms_plus_nextor_usb_1mb.uf2"; WorkingDirectory = $konami2350 },
        @{ Exe = $explorer2350; Options = @("-m1"); Output = "firmware\2350\explorer_1mb_sd_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-m2"); Output = "firmware\2350\explorer_1mb_usb_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-c1"); Output = "firmware\2350\explorer_1mb_sd_carnivore2_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-c2"); Output = "firmware\2350\explorer_1mb_usb_carnivore2_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-r1"); Output = "firmware\2350\explorer_1mb_sd_megaram_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-r2"); Output = "firmware\2350\explorer_1mb_usb_megaram_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-a"); Output = "firmware\2350\explorer_all_nextor.uf2" },
        @{ Exe = $explorer2350; Options = @("-a"); Output = "firmware\2350\explorer_konami_msx1_plus_all_nextor.uf2"; WorkingDirectory = $konami2350 },
        @{ Exe = $yamanooto2350; Options = @("Konami Neo-Ultimate Collection - SMX Team (2025)(Yamanooto).rom"); Output = "firmware\2350\yamanooto_konami_neo_ultimate_collection.uf2"; WorkingDirectory = $yamanootoBuild }
    )

    $entries = [System.Collections.Generic.List[object]]::new()
    foreach ($spec in $specs) {
        $outputPath = Join-Path $Root $spec.Output
        $workingDirectory = if ($spec.ContainsKey("WorkingDirectory")) { $spec.WorkingDirectory } else { $null }
        $entries.Add((Invoke-Uf2Tool -Root $Root -ExePath $spec.Exe -Options $spec.Options -OutputPath $outputPath -WorkingDirectory $workingDirectory))
    }

    Remove-ObsoleteFirmwareUf2Files -Root $Root -FirmwareEntries $entries

    return $entries
}

function Get-LatestChangeLogEntry {
    param(
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Change log was not found: $Path"
    }

    $lines = Get-Content -LiteralPath $Path
    $headingIndex = -1
    for ($index = 0; $index -lt $lines.Count; $index++) {
        if ($lines[$index] -match '^#{1,2}\s+(.+?)\s*$') {
            $headingIndex = $index
            break
        }
    }

    if ($headingIndex -lt 0) {
        throw "Change log has no version heading: $Path"
    }

    $title = $Matches[1].Trim()
    $body = [System.Collections.Generic.List[string]]::new()
    for ($index = $headingIndex + 1; $index -lt $lines.Count; $index++) {
        $line = $lines[$index]
        if ($line -match '^#{1,2}\s+') {
            break
        }
        if ($line -match '^\s*-\s*$') {
            continue
        }
        if ($line.Trim().Length -eq 0 -and $body.Count -eq 0) {
            continue
        }
        $body.Add($line.TrimEnd())
    }

    while ($body.Count -gt 0 -and $body[$body.Count - 1].Trim().Length -eq 0) {
        $body.RemoveAt($body.Count - 1)
    }

    return [pscustomobject]@{
        Title = $title
        Lines = $body
    }
}

function Get-ReleaseChangeLogEntries {
    param(
        [string]$Root
    )

    $paths = @(
        "2040\software\loadrom.pio\docs\log.md",
        "2040\software\multirom.pio\docs\log.md",
        "2350\software\loadrom.pio\docs\log.md",
        "2350\software\multirom.pio\docs\log.md",
        "2350\software\explorer.pio\docs\log.md",
        "2350\software\yamanooto\docs\log.md"
    )

    $entries = [System.Collections.Generic.List[object]]::new()
    foreach ($path in $paths) {
        $entry = Get-LatestChangeLogEntry -Path (Join-Path $Root $path)
        $entry | Add-Member -NotePropertyName Source -NotePropertyValue $path
        $entries.Add($entry)
    }

    return $entries
}

function Get-PreviousReleaseNotesPath {
    param(
        [string]$ZipRoot,
        [string]$CurrentReleaseNotesPath
    )

    if (-not (Test-Path -LiteralPath $ZipRoot -PathType Container)) {
        return $null
    }

    $currentName = [System.IO.Path]::GetFileName($CurrentReleaseNotesPath)
    $previousRelease = Get-ChildItem -LiteralPath $ZipRoot -File -Filter "msx-picoverse-*.txt" |
        Where-Object { $_.Name -ne $currentName -and $_.BaseName -match '^msx-picoverse-\d{8}$' } |
        Sort-Object Name -Descending |
        Select-Object -First 1

    if ($previousRelease) {
        return $previousRelease.FullName
    }

    return $null
}

function Get-ReleaseLineSet {
    param(
        [string]$Path
    )

    $lineSet = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
    if (-not $Path -or -not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        return $lineSet
    }

    foreach ($line in Get-Content -LiteralPath $Path) {
        $normalized = $line.TrimEnd()
        if ($normalized.Length -gt 0) {
            [void]$lineSet.Add($normalized)
        }
    }

    return $lineSet
}

function Test-NewReleaseLine {
    param(
        [string]$Line,
        [System.Collections.Generic.HashSet[string]]$PreviousReleaseLineSet
    )

    if (-not $PreviousReleaseLineSet -or $PreviousReleaseLineSet.Count -eq 0) {
        return $true
    }

    $normalized = $Line.TrimEnd()
    return -not $PreviousReleaseLineSet.Contains($normalized)
}

function Select-NewChangeLogEntries {
    param(
        [System.Collections.Generic.List[object]]$ChangeLogEntries,
        [System.Collections.Generic.HashSet[string]]$PreviousReleaseLineSet
    )

    if (-not $PreviousReleaseLineSet -or $PreviousReleaseLineSet.Count -eq 0) {
        return $ChangeLogEntries
    }

    $newEntries = [System.Collections.Generic.List[object]]::new()
    foreach ($entry in $ChangeLogEntries) {
        $newLines = [System.Collections.Generic.List[string]]::new()
        foreach ($line in $entry.Lines) {
            if ($line.TrimEnd().Length -eq 0) {
                if ($newLines.Count -gt 0) {
                    $newLines.Add($line)
                }
                continue
            }
            if (Test-NewReleaseLine -Line $line -PreviousReleaseLineSet $PreviousReleaseLineSet) {
                $newLines.Add($line)
            }
        }

        while ($newLines.Count -gt 0 -and $newLines[$newLines.Count - 1].Trim().Length -eq 0) {
            $newLines.RemoveAt($newLines.Count - 1)
        }

        if ($newLines.Count -gt 0) {
            $newEntry = [pscustomobject]@{
                Title = $entry.Title
                Lines = $newLines
            }
            $sourceProperty = $entry.PSObject.Properties["Source"]
            if ($sourceProperty) {
                $newEntry | Add-Member -NotePropertyName Source -NotePropertyValue $sourceProperty.Value
            }
            $newEntries.Add($newEntry)
        }
    }

    return $newEntries
}

function Get-RemoteRepositorySlug {
    $originUrl = (git remote get-url origin 2>$null)
    if (-not $originUrl) {
        return $null
    }

    if ($originUrl -match 'github\.com[:/](?<slug>[^/]+/[^/.]+)(?:\.git)?$') {
        return $Matches.slug
    }

    return $null
}

function Get-GitHubCliPath {
    $command = Get-Command gh -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $programFilesRoots = @(
        [Environment]::GetFolderPath("ProgramFiles"),
        [Environment]::GetEnvironmentVariable("ProgramW6432", "Process"),
        [Environment]::GetEnvironmentVariable("ProgramW6432", "Machine"),
        [Environment]::GetEnvironmentVariable("ProgramFiles", "Process"),
        [Environment]::GetEnvironmentVariable("ProgramFiles", "Machine"),
        [Environment]::GetEnvironmentVariable("ProgramFiles(x86)", "Process"),
        [Environment]::GetEnvironmentVariable("ProgramFiles(x86)", "Machine"),
        "C:\Program Files",
        "C:\Program Files (x86)"
    ) | Where-Object { $_ } | Select-Object -Unique

    $localAppDataRoots = @(
        [Environment]::GetFolderPath("LocalApplicationData"),
        [Environment]::GetEnvironmentVariable("LocalAppData", "Process"),
        [Environment]::GetEnvironmentVariable("LocalAppData", "User")
    ) | Where-Object { $_ } | Select-Object -Unique

    $candidates = [System.Collections.Generic.List[string]]::new()
    foreach ($root in $programFilesRoots) {
        $candidates.Add((Join-Path -Path $root -ChildPath "GitHub CLI\gh.exe"))
    }
    foreach ($root in $localAppDataRoots) {
        $candidates.Add((Join-Path -Path $root -ChildPath "Programs\GitHub CLI\gh.exe"))
    }

    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    return $null
}

function New-ReleaseNotes {
    param(
        [string]$ReleaseName,
        [System.Collections.Generic.List[object]]$ChangeLogEntries,
        [System.Collections.Generic.List[object]]$ToolEntries,
        [System.Collections.Generic.List[object]]$FirmwareEntries,
        [System.Collections.Generic.HashSet[string]]$PreviousReleaseLineSet
    )

    $lines = [System.Collections.Generic.List[string]]::new()

    foreach ($entry in $ChangeLogEntries) {
        $lines.Add("## $($entry.Title)")
        $lines.Add("")
        foreach ($line in $entry.Lines) {
            $lines.Add($line)
        }
        $lines.Add("")
    }

    if ($FirmwareEntries.Count -gt 0) {
        $firmwareLines = [System.Collections.Generic.List[string]]::new()
        foreach ($entry in $FirmwareEntries | Sort-Object Hardware, VersionedFile) {
            $optionSuffix = ""
            $optionsProperty = $entry.PSObject.Properties["Options"]
            if ($optionsProperty -and $optionsProperty.Value) {
                $optionSuffix = " $($optionsProperty.Value)"
            }
            $firmwareLine = "- ``$($entry.VersionedFile)``: generated with ``$($entry.Tool) $($entry.ToolVersion)$optionSuffix``."
            if (Test-NewReleaseLine -Line $firmwareLine -PreviousReleaseLineSet $PreviousReleaseLineSet) {
                $firmwareLines.Add($firmwareLine)
            }
        }

        if ($firmwareLines.Count -gt 0) {
        $lines.Add("## Generated UF2 firmware")
        $lines.Add("")
        foreach ($firmwareLine in $firmwareLines) {
            $lines.Add($firmwareLine)
        }
        $lines.Add("")
        }
    }

    if ($ToolEntries.Count -gt 0) {
        $toolLines = [System.Collections.Generic.List[string]]::new()
        foreach ($entry in $ToolEntries | Sort-Object VersionDir, Name) {
            $docSuffix = ""
            if ($entry.DocPath) {
                $docSuffix = " ([manual]($($entry.DocPath)))"
            }
            $toolLine = "- ``$($entry.VersionDir)/$($entry.Name)``: $($entry.Description)$docSuffix"
            if (Test-NewReleaseLine -Line $toolLine -PreviousReleaseLineSet $PreviousReleaseLineSet) {
                $toolLines.Add($toolLine)
            }
        }

        if ($toolLines.Count -gt 0) {
        $lines.Add("## Included Windows tools")
        $lines.Add("")
        foreach ($toolLine in $toolLines) {
            $lines.Add($toolLine)
        }
        $lines.Add("")
        }
    }

    $packageLineCandidates = @(
        "- ``exe/``: Windows command-line tools grouped by PicoVerse hardware family.",
        "- ``firmware/``: ready-to-flash UF2 images generated for this release, including versioned filenames that identify the producing tool.",
        "- ``docs/``: user manuals and technical documentation for the released features.",
        "- ``hardware/``: current hardware BOMs and related manufacturing files."
    )

    $packageLines = [System.Collections.Generic.List[string]]::new()
    foreach ($packageLine in $packageLineCandidates) {
        if (Test-NewReleaseLine -Line $packageLine -PreviousReleaseLineSet $PreviousReleaseLineSet) {
            $packageLines.Add($packageLine)
        }
    }

    if ($packageLines.Count -gt 0) {
        $lines.Add("## Package contents")
        $lines.Add("")
        foreach ($packageLine in $packageLines) {
            $lines.Add($packageLine)
        }
    }

    return ($lines -join [Environment]::NewLine)
}

function New-CommitMessage {
    param(
        [string]$ReleaseName,
        [System.Collections.Generic.List[object]]$ChangeLogEntries
    )

    $lines = [System.Collections.Generic.List[string]]::new()

    $lines.Add("Release $ReleaseName")
    $lines.Add("")

    foreach ($entry in $ChangeLogEntries) {
        $lines.Add($entry.Title)
        $lines.Add("")
        foreach ($line in $entry.Lines) {
            $lines.Add($line)
        }
        $lines.Add("")
    }

    while ($lines.Count -gt 0 -and $lines[$lines.Count - 1].Trim().Length -eq 0) {
        $lines.RemoveAt($lines.Count - 1)
    }

    return ($lines -join [Environment]::NewLine)
}

function Invoke-ReleaseCommit {
    param(
        [string]$Root,
        [string]$ReleaseName,
        [System.Collections.Generic.List[object]]$ChangeLogEntries
    )

    $messagePath = Join-Path $Root "build\package-staging\commit-message.txt"
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $messagePath) | Out-Null
    Set-Content -LiteralPath $messagePath -Value (New-CommitMessage -ReleaseName $ReleaseName -ChangeLogEntries $ChangeLogEntries)

    $startDirectory = (Get-Location).Path
    try {
        Set-Location -LiteralPath $Root

        & git add -A
        if ($LASTEXITCODE -ne 0) {
            throw "git add failed with exit code $LASTEXITCODE."
        }

        & git diff --cached --quiet --exit-code
        $diffExitCode = $LASTEXITCODE
        if ($diffExitCode -eq 0) {
            Write-Host "No staged changes to commit."
            return
        }
        if ($diffExitCode -ne 1) {
            throw "git diff --cached failed with exit code $diffExitCode."
        }

        & git commit --cleanup=whitespace -F $messagePath
        if ($LASTEXITCODE -ne 0) {
            throw "git commit failed with exit code $LASTEXITCODE."
        }
    } finally {
        Set-Location -LiteralPath $startDirectory
    }
}

$root = (git rev-parse --show-toplevel 2>$null)
if (-not $root) {
    $root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
}
$root = $root.TrimEnd('\', '/')

$staging = Join-Path $root "build/package-staging"
$zipRoot = Join-Path $root "releases"

if ($Clean) {
    if (Test-Path $staging) {
        Remove-Item -Recurse -Force $staging
    }
    exit 0
}

$tag = (Get-Date -Format "yyyyMMdd")

$exeRoot = Join-Path $staging "exe"
$docsRoot = Join-Path $staging "docs"
$firmwareRoot = Join-Path $staging "firmware"
$hardwareRoot = Join-Path $staging "hardware"
$outZip = Join-Path $zipRoot ("msx-picoverse-" + $tag + ".zip")
$zipBase = [System.IO.Path]::GetFileNameWithoutExtension($outZip)
$outTxt = Join-Path $zipRoot ($zipBase + ".txt")
$releaseName = $zipBase

if (Test-Path $staging) {
    Remove-Item -Recurse -Force $staging
}
New-Item -ItemType Directory -Force -Path $exeRoot, $docsRoot, $firmwareRoot, $hardwareRoot, $zipRoot | Out-Null

$previousReleaseNotesPath = Get-PreviousReleaseNotesPath -ZipRoot $zipRoot -CurrentReleaseNotesPath $outTxt
$previousReleaseLineSet = Get-ReleaseLineSet -Path $previousReleaseNotesPath
$changeLogEntries = Select-NewChangeLogEntries -ChangeLogEntries (Get-ReleaseChangeLogEntries -Root $root) -PreviousReleaseLineSet $previousReleaseLineSet
$firmwareEntries = New-FirmwareUf2Files -Root $root

$searchRoots = @(
    (Join-Path $root "2040\software"),
    (Join-Path $root "2350\software")
)

$exeFiles = Get-ChildItem -Recurse -File -Filter *.exe -Path $searchRoots |
    Where-Object { $_.DirectoryName -match "[\\/]dist$" }

foreach ($exe in $exeFiles) {
    $versionDir = "other"
    if ($exe.FullName -match "[\\/]2040[\\/]software[\\/]") {
        $versionDir = "2040"
    } elseif ($exe.FullName -match "[\\/]2350[\\/]software[\\/]") {
        $versionDir = "2350"
    }
    $dest = Join-Path (Join-Path $exeRoot $versionDir) $exe.Name
    New-Item -ItemType Directory -Force -Path (Split-Path $dest) | Out-Null
    Copy-Item -Force $exe.FullName $dest
}

$toolEntries = [System.Collections.Generic.List[object]]::new()
foreach ($exe in Get-ChildItem -Recurse -File -Filter *.exe -Path $exeRoot) {
    $versionDir = Split-Path $exe.DirectoryName -Leaf
    $toolEntries.Add([pscustomobject]@{
        VersionDir   = $versionDir
        Name         = $exe.Name
        Description  = Get-ToolDescription -VersionDir $versionDir -FileName $exe.Name
        DocPath      = Get-ToolDocPath -VersionDir $versionDir -FileName $exe.Name
    })
}

$docsPath = Join-Path $root "docs"
if (Test-Path $docsPath) {
    Copy-Item -Recurse -Force (Join-Path $docsPath "*") $docsRoot
}

$repoFirmwareRoot = Join-Path $root "firmware"
if (Test-Path $repoFirmwareRoot) {
    Copy-Item -Recurse -Force (Join-Path $repoFirmwareRoot "*") $firmwareRoot
}

$hardware2040 = Join-Path $root "2040\hardware"
if (Test-Path $hardware2040) {
    $dest = Join-Path $hardwareRoot "2040"
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    Copy-Item -Recurse -Force (Join-Path $hardware2040 "*") $dest
}

$hardware2350 = Join-Path $root "2350\hardware"
if (Test-Path $hardware2350) {
    $dest = Join-Path $hardwareRoot "2350"
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    Copy-Item -Recurse -Force (Join-Path $hardware2350 "*") $dest
}

if (Test-Path $outZip) {
    Remove-Item -Force $outZip
}
Compress-Archive -Path (Join-Path $staging "*") -DestinationPath $outZip

$releaseNotes = New-ReleaseNotes -ReleaseName $releaseName -ChangeLogEntries $changeLogEntries -ToolEntries $toolEntries -FirmwareEntries $firmwareEntries -PreviousReleaseLineSet $previousReleaseLineSet
Set-Content -LiteralPath $outTxt -Value $releaseNotes

if ($Commit) {
    Invoke-ReleaseCommit -Root $root -ReleaseName $releaseName -ChangeLogEntries $changeLogEntries
}

if (-not $Release) {
    exit 0
}

$gh = Get-GitHubCliPath
if (-not $gh) {
    throw "GitHub CLI ('gh') was not found. Install it and run 'gh auth login' before using -Release."
}

$repositorySlug = $Repository
if (-not $repositorySlug) {
    $repositorySlug = Get-RemoteRepositorySlug
}
if (-not $repositorySlug) {
    throw "Could not determine the GitHub repository slug. Pass -Repository owner/repo or configure the origin remote."
}

$existingRelease = $null
try {
    $existingRelease = & $gh release view $releaseName --repo $repositorySlug --json tagName 2>$null
} catch {
    $existingRelease = $null
}

$assetSpecs = @(
    "$outZip#$(Get-ReleaseAssetLabel -FilePath $outZip)",
    "$outTxt#$(Get-ReleaseAssetLabel -FilePath $outTxt)"
)

if ($existingRelease) {
    & $gh release edit $releaseName --repo $repositorySlug --title $releaseName --notes-file $outTxt
    & $gh release upload $releaseName --repo $repositorySlug --clobber @assetSpecs
} else {
    & $gh release create $releaseName --repo $repositorySlug --title $releaseName --notes-file $outTxt @assetSpecs
}
