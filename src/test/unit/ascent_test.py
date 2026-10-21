#!/usr/bin/env python3
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[3]
for stem, relative in (
    ("ascent", "src/main/drivers/ascent.c"),
    ("ascent_slave", "tools/SPI_LOG/ESP32blackboxtest/main/ascent_slave.c"),
):
    source = ROOT / relative
    test = pathlib.Path(__file__).with_name(stem + "_test.c")
    stub = pathlib.Path(__file__).with_name(stem + "_test_stubs.h")
    with tempfile.TemporaryDirectory(prefix="ascent-test-") as directory:
        temp = pathlib.Path(directory)
        for name in re.findall(r'^#include "([^"]+)"', source.read_text(), re.M):
            path = temp / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(('#include "' + str(stub) + '"\n'))
        executable = temp / "ascent-test"
        subprocess.run([
            "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=address,undefined", "-g", "-I", str(temp),
            "-fsanitize-address-use-after-return=always",
            str(test), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
