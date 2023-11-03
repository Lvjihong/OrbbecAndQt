; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

#define MyAppName "DBN"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "LJH"
#define MyAppURL "https://www.example.com/"
#define MyAppExeName "OrbbecDemo1.1.exe"
#define MyAppAssocName MyAppName + " File"
#define MyAppAssocExt ".myp"
#define MyAppAssocKey StringChange(MyAppAssocName, " ", "") + MyAppAssocExt

[Setup]
; NOTE: The value of AppId uniquely identifies this application. Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{228C2B52-3C2A-4B0B-B973-DADF43DA9649}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={autopf}\{#MyAppName}
ChangesAssociations=yes
DisableProgramGroupPage=yes
; Uncomment the following line to run in non administrative install mode (install for current user only.)
;PrivilegesRequired=lowest
OutputDir=F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\Output
OutputBaseFilename=DBNsetup
Compression=lzma
SolidCompression=yes
WizardStyle=modern

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\OrbbecSDK.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\c10.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\caffe2_module_test_dynamic.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\libiomp5md.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\libiompstubs5md.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\MicrosoftVisualStudio\Source\Repos\OrbbecDemo1.1\x64\Release\torch.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\Qt\Qt5.14.2\5.14.2\msvc2017_64\bin\Qt5Core.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\Qt\Qt5.14.2\5.14.2\msvc2017_64\bin\Qt5Gui.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\Qt\Qt5.14.2\5.14.2\msvc2017_64\bin\Qt5Widgets.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\opencv\opencv\build\x64\vc16\bin\opencv_world480.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "C:\Windows\System32\kernel32.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "C:\Windows\SysWOW64\shell32.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "F:\Qt\Qt5.14.2\5.14.2\msvc2017_64\plugins\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Registry]
Root: HKA; Subkey: "Software\Classes\{#MyAppAssocExt}\OpenWithProgids"; ValueType: string; ValueName: "{#MyAppAssocKey}"; ValueData: ""; Flags: uninsdeletevalue
Root: HKA; Subkey: "Software\Classes\{#MyAppAssocKey}"; ValueType: string; ValueName: ""; ValueData: "{#MyAppAssocName}"; Flags: uninsdeletekey
Root: HKA; Subkey: "Software\Classes\{#MyAppAssocKey}\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\{#MyAppExeName},0"
Root: HKA; Subkey: "Software\Classes\{#MyAppAssocKey}\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\{#MyAppExeName}"" ""%1"""
Root: HKA; Subkey: "Software\Classes\Applications\{#MyAppExeName}\SupportedTypes"; ValueType: string; ValueName: ".myp"; ValueData: ""

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

