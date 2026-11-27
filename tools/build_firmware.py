"""Build the STM32G474 flash image with the Arm GNU Toolchain."""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys


def run(tool, options, cwd):
    subprocess.run([str(tool), *(str(option) for option in options)],
                   cwd=cwd, check=True)


def build(gcc, project, output):
    name = project.name
    tool_dir = gcc.parent
    suffix = ".exe" if sys.platform == "win32" else ""
    objcopy = tool_dir / ("arm-none-eabi-objcopy" + suffix)
    size = tool_dir / ("arm-none-eabi-size" + suffix)
    for tool in (objcopy, size):
        if not tool.is_file():
            raise SystemExit(f"Missing Arm GNU Toolchain program: {tool.name}")

    cpu = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    include_dirs = [
        "Core/Inc",
        "Drivers/STM32G4xx_HAL_Driver/Inc",
        "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
        "Drivers/CMSIS/Device/ST/STM32G4xx/Include",
        "Drivers/CMSIS/Include",
    ]
    c_flags = [
        "-std=gnu11", "-Os", "-g3", "-ffunction-sections", "-fdata-sections",
        "-Wall", "--specs=nano.specs", "-DUSE_HAL_DRIVER", "-DSTM32G474xx",
        *("-I" + directory for directory in include_dirs),
    ]
    sources = sorted(project.glob("Core/Src/*.c"))
    sources += sorted(project.glob("Drivers/STM32G4xx_HAL_Driver/Src/*.c"))
    sources.append(project / "Core/Startup/startup_stm32g474rbtx.s")

    objects = []
    for source in sources:
        relative_source = source.relative_to(project)
        obj = output / "obj" / relative_source.with_suffix(".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        flags = c_flags if source.suffix == ".c" else ["-g3", "-x", "assembler-with-cpp"]
        print(f"Compiling {relative_source.as_posix()}", flush=True)
        run(gcc, [*cpu, *flags, "-c", relative_source, "-o", obj], project)
        objects.append(obj)

    elf = output / f"{name}.elf"
    run(gcc, [
        *cpu, *objects, "-T", "STM32G474RBTX_FLASH.ld",
        "--specs=nano.specs", "--specs=nosys.specs", "-static",
        "-Wl,--gc-sections", f"-Wl,-Map={elf.with_suffix('.map')}",
        "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group",
        "-o", elf,
    ], project)
    run(objcopy, ["-O", "ihex", elf, elf.with_suffix(".hex")], project)
    run(size, [elf], project)
    print(f"Built build/{elf.name} and build/{elf.with_suffix('.hex').name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", default="arm-none-eabi-gcc",
                        help="compiler on PATH or the full path to arm-none-eabi-gcc")
    args = parser.parse_args()
    gcc = shutil.which(args.gcc)
    if gcc is None:
        parser.error("Arm GCC unavailable. Add it to PATH or specify --gcc.")
    repo = Path(__file__).resolve().parents[1]
    try:
        build(Path(gcc).absolute(), repo / "STM32G474RBT3_FW", repo / "build")
    except subprocess.CalledProcessError as exc:
        sys.exit(exc.returncode)


if __name__ == "__main__":
    main()
