param(
    [string]$ArduinoCli = 'arduino-cli',
    [string[]]$Boards = @(
        'arduino:avr:uno',
        'arduino:avr:mega',
        'arduino:sam:arduino_due_x',
        'arduino:renesas_uno:minima',
        'arduino:renesas_uno:unor4wifi',
        'arduino:samd:arduino_zero_native'
    )
)

$ErrorActionPreference = 'Stop'
$libraryRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testRoot = Join-Path ([IO.Path]::GetTempPath()) ('furuta-sampling-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testRoot | Out-Null

$furutaBody = @'
void controlStep() {}
void setup() {
  FurutaShield.begin();
  Sampling.interrupt(controlStep);
  Sampling.period(10000);
  FurutaShield.actuatorWrite(1.0f);
}
void loop() { FurutaShield.sensorRead(); }
'@
$optoBody = @'
void controlStep() {}
void setup() {
  OptoShield.begin();
  Sampling.interrupt(controlStep);
  Sampling.period(10000);
}
void loop() { OptoShield.actuatorWrite(OptoShield.sensorRead()); }
'@
$cases = [ordered]@{
    FurutaFirst = "#include <FurutaShield.h>`n#include <SamplingServo.h>`n" + $furutaBody
    ServoFirst = "#include <AutomationShield.h>`n#include <SamplingServo.h>`n#include <FurutaShield.h>`n" + $furutaBody
    OtherShield = "#include <OptoShield.h>`n#include <Sampling.h>`n" + $optoBody
    OtherShieldServo = "#include <OptoShield.h>`n#include <SamplingServo.h>`n" + $optoBody
    ConflictFirst = "#include <FurutaShield.h>`n#include <Sampling.h>`n" + $furutaBody
    ConflictLast = "#include <AutomationShield.h>`n#include <Sampling.h>`n#include <FurutaShield.h>`n" + $furutaBody
}

foreach ($case in $cases.GetEnumerator()) {
    $sketchDir = Join-Path $testRoot $case.Key
    New-Item -ItemType Directory -Path $sketchDir | Out-Null
    [IO.File]::WriteAllText((Join-Path $sketchDir ($case.Key + '.ino')), $case.Value)
    if ($case.Key -eq 'FurutaFirst') {
        # Ensure additional translation units can use the API without duplicate ISRs.
        [IO.File]::WriteAllText((Join-Path $sketchDir 'helper.cpp'),
            "#include <furuta/FurutaClass.h>`nvoid stopFuruta() { FurutaShield.emergStop(); }`n")
    }
}

foreach ($board in $Boards) {
    foreach ($case in $cases.GetEnumerator()) {
        $buildDir = Join-Path $testRoot ('build-' + $board.Replace(':', '-') + '-' + $case.Key)
        $logPath = $buildDir + '.log'
        # Windows PowerShell treats native stderr as an error even for expected
        # compiler diagnostics. Inspect the exit code and log ourselves.
        $ErrorActionPreference = 'Continue'
        try {
            & $ArduinoCli compile --fqbn $board --library $libraryRoot --build-path $buildDir (Join-Path $testRoot $case.Key) *> $logPath
            $compileExit = $LASTEXITCODE
        } finally {
            $ErrorActionPreference = 'Stop'
        }
        $log = [IO.File]::ReadAllText($logPath)
        if ($case.Key.StartsWith('Conflict')) {
            if ($compileExit -eq 0 -or $log -notmatch 'Use SamplingServo.h for control-loop sampling') {
                throw "Expected timer conflict diagnostic: $board / $($case.Key). See $logPath"
            }
        } elseif ($compileExit -ne 0) {
            throw "Compile failed: $board / $($case.Key). See $logPath"
        }
        Write-Output "PASS $board / $($case.Key)"
    }
}
Write-Output "Build logs: $testRoot"
