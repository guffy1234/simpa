# Ensure directories are created
$instanceName = "Ubuntu-24.04-Simpa"
$imageFile = "ubuntu-noble-wsl-amd64-wsl.rootfs.tar.gz"
$envScriptName = "install_ardupilot_ros2-jazzy_gazebo-harmonic.sh"
$imageUrl = "https://cloud-images.ubuntu.com/wsl/releases/24.04/current/ubuntu-noble-wsl-amd64-wsl.rootfs.tar.gz"

$basePath = "c:\Simpa"
$wslPath = "$basePath\wsl"
$installPath = "$wslPath\$instanceName"
$imagePath = "$wslPath\$imageFile"
$envScriptPathWin = "$basePath\scripts\$envScriptName"
$envScriptPathMnt = "/mnt/c/Simpa/scripts/$envScriptName"
$envScriptPathWsl = "/home/simuser/install_simpa.sh"

# Create directories
New-Item -ItemType Directory -Force -Path $wslPath | Out-Null

# Change to the WSL directory
if (-not (Test-Path "$wslPath")) {
    New-Item -Path "$wslPath" -ItemType Directory
}
Set-Location -Path $wslPath

# Check if Ubuntu image already exists
if (Test-Path $imageFile) {
    Write-Host "Ubuntu image already exists at $imageFile. Skipping download."
} else {
    # Download Ubuntu (x86_64; change to arm64 if needed)
    Write-Host "Downloading Ubuntu image..."
    Invoke-WebRequest -Uri $imageUrl -OutFile $imageFile -ErrorAction Stop
}

# Ensure WSL2 is set as default
Write-Host "Setting WSL2 as default version..."
wsl --set-default-version 2

# Import the WSL instance
Write-Host "Importing WSL instance $instanceName..."
wsl --import $instanceName $installPath $imageFile --version 2

# Pause to allow instance creation to complete
Write-Host "Waiting 10 seconds for instance creation..."
Start-Sleep -Seconds 10

# Verify the instance was created
Write-Host "Verifying WSL instance $instanceName..."
$wslList = wsl --list --quiet
if ($null -eq $wslList) {
    Write-Host "Failed to list WSL instances. Ensure WSL is installed and functional."
    exit 1
}
# Debug: Print raw output for inspection
Write-Host "Raw output of 'wsl --list --quiet':"
Write-Host ($wslList -join "`n")
# Check for exact match of $instanceName
if ($wslList | Where-Object { $_ -eq "$instanceName" }) {
    Write-Host "WSL instance $instanceName created successfully."
} else {
    Write-Host "Failed to create WSL instance $instanceName. Output of 'wsl --list --quiet':"
    Write-Host ($wslList -join "`n")
    exit 1
}

# Start the WSL instance and configure it
Write-Host "Configuring WSL instance $instanceName..."

# Create simuser
Write-Host "Creating simuser..."
wsl -d $instanceName -u root -- bash -c @"
set -e
if id simuser &>/dev/null; then
    echo 'User simuser already exists. Skipping user creation.'
else
    adduser --gecos '' --disabled-password simuser
    echo 'User simuser created.'
fi
echo 'simuser ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/simuser
chmod 0440 /etc/sudoers.d/simuser
if visudo -c; then
    echo 'Sudo configuration for simuser is valid.'
else
    echo 'Sudo configuration failed.'
    exit 1
fi
"@

# Set simuser as default user
Write-Host "Setting simuser as default user..."
wsl -d $instanceName -u root -- bash -c @"
set -e
if ! id simuser &>/dev/null; then
    echo 'User simuser does not exist.'
    exit 1
fi
cat > /etc/wsl.conf << EOF
[user]
default=simuser
EOF
"@

# Terminate the WSL instance to apply changes
Write-Host "Terminating WSL instance to apply changes..."
wsl --terminate $instanceName

# Restart the WSL instance as simuser
Write-Host "Restarting WSL instance as simuser..."
wsl -d $instanceName -u simuser -- whoami
$whoami = wsl -d $instanceName -u simuser -- whoami
if ($whoami -eq "simuser") {
    Write-Host "Successfully logged in as simuser."
} else {
    Write-Host "Failed to log in as simuser. Current user: $whoami"
    exit 1
}

# Check if the environment setup script exists
if (-Not (Test-Path $envScriptPathWin)) {
    Write-Host "Environment setup script $envScriptPathWin not found."
    exit 1
}

# Copy the environment setup script to simuser's home directory
Write-Host "Copying $envScriptPathWin to $envScriptPathWsl..."
wsl -d $instanceName -u simuser -- bash -c "mkdir -p ~ && cp $envScriptPathMnt $envScriptPathWsl"

# Make the script executable
Write-Host "Making $envScriptPathWsl executable..."
wsl -d $instanceName -u simuser -- bash -c @"
set -e
chmod +x $envScriptPathWsl
"@

Write-Host "Setup complete! WSL instance $instanceName is ready."
Write-Host "Start it with 'wsl -d $instanceName'."
Write-Host "To run the environment setup script, run: $envScriptPathWsl"

Set-Location -Path $basePath