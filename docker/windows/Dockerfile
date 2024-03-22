# escape=`
# this is the Dockerfile version only
LABEL image-version="1.0" 
ARG WINDOWS_SERVER_VERSION=2022

FROM mcr.microsoft.com/windows/servercore:ltsc${WINDOWS_SERVER_VERSION}
SHELL ["cmd", "/S", "/C"]

#install VS-Buildtools https://learn.microsoft.com/de-de/visualstudio/install/build-tools-container
RUN `
    # Download the Build Tools bootstrapper.
    curl -SL --output vs_buildtools.exe https://aka.ms/vs/17/release/vs_buildtools.exe `
    `
    # Install Build Tools
    && (start /w vs_buildtools.exe --quiet --wait --norestart --nocache --includeRecommended `
        --installPath "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools" `
		--add Microsoft.VisualStudio.Workload.VCTools `
        || IF "%ERRORLEVEL%"=="3010" EXIT 0) `
    `
    # Cleanup
    && del /q vs_buildtools.exe

# install CUDA Compiler
RUN `
    # Download the Build Tools bootstrapper.
    curl -SL --output cuda_12.3.2_546.12_windows.exe https://developer.download.nvidia.com/compute/cuda/12.3.2/local_installers/cuda_12.3.2_546.12_windows.exe `
    `
    # Install CUDA with nvcc and runtime libs
    && (start /w cuda_12.3.2_546.12_windows.exe -s `
        #nvcc_12.3 `
		#cudart_12.3 `
        #visual_studio_integration_12.3 `
        || IF "%ERRORLEVEL%"=="3010" EXIT 0) `
    `
	# Copy Build intregartion files, because cuda installation does not do this for BuildTools
	&& copy "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3\extras\visual_studio_integration\MSBuildExtensions\*" "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Microsoft\VC\v170\BuildCustomizations" /y `
    # Cleanup
    && del /q cuda_12.3.2_546.12_windows.exe `
	# Add to PATH
	&& setx PATH "%PATH%;C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3\bin"

# install git
RUN `
	curl -SL --output Git-2.44.0-64-bit.exe https://github.com/git-for-windows/git/releases/download/v2.44.0.windows.1/Git-2.44.0-64-bit.exe `
	&& Git-2.44.0-64-bit.exe /VERYSILENT /NORESTART /NOCANCEL /SP- /CLOSEAPPLICATIONS /RESTARTAPPLICATIONS /COMPONENTS="icons,ext\reg\shellhere,assoc,assoc_sh" `
	`
    # Cleanup
    && del /q Git-2.44.0-64-bit.exe `
	# Add to PATH
	&& setx PATH "%PATH%;C:\Program Files\Git\bin"
	
# copy vcpkg, Dockerfile expects local prepared vcpkg directory. See README for vcpkg deps. Use :x64-windows triplet and delete the vcpkg/downloads/ and vcpkg/buildtrees/ directories to reduce size.
COPY vcpkg c:/vcpkg
RUN setx VCPKG_ROOT C:\VCPKG


# Entry point for developer command prompt and PowerShell
ENTRYPOINT ["C:\\Program Files (x86)\\Microsoft Visual Studio\\2022\\BuildTools\\VC\\Auxiliary\\Build\\vcvars64.bat", "&&", "powershell.exe", "-NoLogo", "-ExecutionPolicy", "Bypass"]