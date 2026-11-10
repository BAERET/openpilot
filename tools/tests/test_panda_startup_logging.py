"""Exercise the production serial-log router without Panda/cereal dependencies."""
import os
from pathlib import Path
import shlex
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_startup_diagnostics_reach_warning_log(tmp_path):
  source = (ROOT / "openpilot/selfdrive/pandad/pandad.cc").read_text(encoding="utf-8")
  start = source.index("void log_panda_serial(")
  end = source.index("\nvoid pandad_run(", start)
  harness = r'''
#include <cassert>
#include <cstdarg>
#include <string>
#include <vector>
enum { CLOUDLOG_ERROR=40, CLOUDLOG_WARNING=30, CLOUDLOG_DEBUG=10 };
struct Entry { int level; std::string source, text; };
static std::vector<Entry> entries;
static void cloudlog_e(int level, const char *source, int line, const char *fn, const char *format, ...) {
  (void)line; (void)fn; (void)format;
  va_list args; va_start(args, format);
  const char *text = va_arg(args, const char *);
  entries.push_back({level, source, text});
  va_end(args);
}
''' + source[start:end] + r'''
int main() {
  log_panda_serial(0, "ordinary debug\r\ncan_startup_diag: v=1 bus=00000002\r\n");
  assert(entries.size()==2);
  assert(entries[0].level==CLOUDLOG_DEBUG && entries[0].text=="ordinary debug");
  assert(entries[1].level==CLOUDLOG_WARNING && entries[1].source=="panda[0]");
  assert(entries[1].text=="can_startup_diag: v=1 bus=00000002");
  entries.clear();
  log_panda_serial(1, "SPI: CAN NACK\n");
  assert(entries.size()==1 && entries[0].level==CLOUDLOG_WARNING);
  entries.clear();
  log_panda_serial(0, "Register 0x123 fault\n");
  assert(entries.size()==1 && entries[0].level==CLOUDLOG_ERROR);
}
'''
  test_source = tmp_path / "startup_logging.cpp"
  executable = tmp_path / ("startup_logging.exe" if os.name == "nt" else "startup_logging")
  test_source.write_text(harness, encoding="utf-8")
  compiler = shlex.split(os.environ.get("CXX", "c++"))
  subprocess.run([*compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                  str(test_source), "-o", str(executable)], check=True)
  subprocess.run([str(executable)], check=True, timeout=10)
