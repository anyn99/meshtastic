#!/usr/bin/env python3
"""Build-/Upload-Targets je ALMEMO-Variante in Eclipses .cproject eintragen.

Warum: ohne diese Targets schaltet man die Variante ueber `default_envs` in
platformio.ini um -- das aendert eine verfolgte Datei und macht den
Arbeitsbaum dirty (was ueber APP_DIRTY bis aufs Display durchschlaegt). Mit
`-e <env>` am Target ist `default_envs` wirkungslos und die Datei bleibt, wie
sie ist.

.cproject ist gitignoriert UND wird von `pio project init --ide eclipse`
komplett neu geschrieben. Deshalb liegt der Generator hier im Repo: nach einem
Projekt-Refresh ist es ein Aufruf statt Handarbeit.

Idempotent: eigene Eintraege werden an "-e xiao_nrf52840_almemo" im
<buildTarget> erkannt und vor dem Einfuegen entfernt. Die globalen Targets
(Build, Clean, List Devices, Upload, CompileDB) bleiben unberuehrt.

ECLIPSE VORHER SCHLIESSEN -- CDT haelt das Projektmodell im Speicher und
schreibt .cproject beim Beenden sonst wieder zurueck.

Aufruf:  python3 bin/eclipse-targets.py [--dry-run]
"""

import os
import re
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[0])))
CPROJECT = os.path.join(REPO, ".cproject")
VARIANT_INI = os.path.join(REPO, "variants", "nrf52840", "seeed_xiao_nrf52840_kit", "platformio.ini")

# Nur Kosmetik: kurze Namen fuer die Build-Targets-Ansicht. Die Reihenfolge
# hier ist die Einfuegereihenfolge; Envs ohne Eintrag bekommen einen aus dem
# Namen abgeleiteten Titel und landen hinten.
LABELS = [
    ("xiao_nrf52840_almemo_sensor_receiver", "Receiver D6"),
    ("xiao_nrf52840_almemo_sender_D7_oled", "Node D7+OLED"),
    ("xiao_nrf52840_almemo_sensor_sender_D7", "Node D7"),
    ("xiao_nrf52840_almemo_sensor_sender", "Node D6"),
    ("xiao_nrf52840_almemo_sender_nosensor", "Node D6 emulated"),
    ("xiao_nrf52840_almemo_sender_epaper_mux", "Node D6 Mux+EPaper"),
]

TARGET = """\t\t\t<target name="{name}" path="" targetID="org.eclipse.cdt.build.MakeTargetBuilder">
\t\t\t\t<buildCommand>platformio</buildCommand>
\t\t\t\t<buildArguments>-f -c eclipse</buildArguments>
\t\t\t\t<buildTarget>{args}</buildTarget>
\t\t\t\t<stopOnError>true</stopOnError>
\t\t\t\t<useDefaultCommand>false</useDefaultCommand>
\t\t\t\t<runAllBuilders>true</runAllBuilders>
\t\t\t</target>
"""


def envs():
    """ALMEMO-Envs aus der Variant-ini, damit eine neue Variante mitkommt."""
    text = open(VARIANT_INI, encoding="utf-8").read()
    found = re.findall(r"^\[env:(xiao_nrf52840_almemo[A-Za-z0-9_]*)\]", text, re.M)
    known = [e for e, _ in LABELS]
    ordered = [e for e in known if e in found] + [e for e in found if e not in known]
    labels = dict(LABELS)
    return [(e, labels.get(e, e.replace("xiao_nrf52840_almemo_", ""))) for e in ordered]


def main():
    dry = "--dry-run" in sys.argv
    src = open(CPROJECT, encoding="utf-8").read()

    start = src.find("\t\t<buildTargets>")
    end = src.find("\t\t</buildTargets>")
    if start < 0 or end < 0:
        sys.exit("ABBRUCH: <buildTargets> nicht gefunden -- ist .cproject von PlatformIO erzeugt?")

    head, block, tail = src[:start], src[start:end], src[end:]

    # Eigene Eintraege entfernen (Idempotenz), fremde behalten.
    kept, dropped = [], 0
    for m in re.finditer(r"\t\t\t<target .*?\n\t\t\t</target>\n", block, re.S):
        if "-e xiao_nrf52840_almemo" in m.group(0):
            dropped += 1
        else:
            kept.append(m.group(0))

    added = []
    for env, label in envs():
        added.append(TARGET.format(name="%s: Build" % label, args="run -e %s" % env))
        added.append(TARGET.format(name="%s: Upload" % label, args="run -e %s --target upload" % env))

    out = head + "\t\t<buildTargets>\n" + "".join(kept) + "".join(added) + tail

    print("%d eigene Eintraege entfernt, %d behalten, %d neu:" % (dropped, len(kept), len(added)))
    for env, label in envs():
        print("   %-16s -> run -e %s [ + --target upload ]" % (label + ":", env))

    if dry:
        print("\n--dry-run: nichts geschrieben")
        return
    shutil.copyfile(CPROJECT, CPROJECT + ".bak")
    open(CPROJECT, "w", encoding="utf-8").write(out)
    print("\n.cproject geschrieben (Sicherung: .cproject.bak)")
    print("Eclipse neu starten, dann stehen die Targets in der Build-Targets-Ansicht.")


main()
