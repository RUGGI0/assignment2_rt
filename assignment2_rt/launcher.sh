#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN="$SCRIPT_DIR/run_assignment.sh"
SESSION="assignment2_rt"
PIDFILE="/tmp/${SESSION}_term.pid"

# Gradient colors
C1='\e[31m'   # red
C2='\e[33m'   # yellow
C3='\e[32m'   # green
C4='\e[36m'   # cyan
C5='\e[34m'   # blue
RESET='\e[0m'

kill_tmux_terminal() {
  if [ -f "$PIDFILE" ]; then
    PID="$(cat "$PIDFILE" 2>/dev/null || true)"
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
      kill "$PID" 2>/dev/null || true
    fi
    rm -f "$PIDFILE"
  fi
}

open_tmux_terminal() {
  x-terminal-emulator -e bash -lc "
    cd /home/ubuntu/ros2_workshop;
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true;
    source install/setup.bash 2>/dev/null || true;
    tmux attach -t $SESSION
  " >/dev/null 2>&1 &
  echo $! > "$PIDFILE"
}

# Avvia/ricrea la sessione tmux SENZA attaccarsi (non blocca il launcher)
start_tmux_session_detached() {
  # run_assignment.sh crea la sessione e fa attach.
  # Noi lo eseguiamo in un tmux "bootstrap" separato e lo stacchiamo subito.
  tmux kill-session -t "$SESSION" 2>/dev/null || true

  # Avvia RUN in background e disaccoppia: run_assignment creerà la sessione e farà attach,
  # ma non bloccherà questo launcher.
  nohup bash -lc "$RUN" >/tmp/${SESSION}_run.log 2>&1 &
}

while true; do
  clear

  echo -e "${C1}                          ___           ___           ___                    ${RESET}"
  echo -e "${C1}                         /\\  \\         /\\  \\         /\\  \\                   ${RESET}"
  echo -e "${C1}                        /::\\  \\       /::\\  \\       /::\\  \\                  ${RESET}"
  echo -e "${C2}                       /:/\\:\\  \\     /:/\\:\\  \\     /:/\\:\\  \\                 ${RESET}"
  echo -e "${C2}                      /::\\ \\:\\  \\   /:/  \\:\\  \\   /:/  \\:\\  \\                ${RESET}"
  echo -e "${C3}                     /:/\\:\\ \\:\\__\\ /:/__/_\\:\\__\\ /:/__/ \\:\\__\\               ${RESET}"
  echo -e "${C3}                     \\/_|::\\/:/  / \\:\\  /\\ \\/__/ \\:\\  \\ /:/  /               ${RESET}"
  echo -e "${C4}                        |:|::/  /   \\:\\ \\:\\__\\    \\:\\  /:/  /                ${RESET}"
  echo -e "${C4}                        |:|\\/__/     \\:\\/:/  /     \\:\\/:/  /                 ${RESET}"
  echo -e "${C5}                        |:|  |        \\::/  /       \\::/  /                  ${RESET}"
  echo -e "${C5}                         \\|__|         \\/__/         \\/__/                   ${RESET}"

  echo ""
  echo "                           =============================="
  echo "                                 ASSIGNMENT TOOL"
  echo "                           =============================="
  echo ""
  echo "                                 S = Start / Attach"
  echo "                                 R = Restart"
  echo "                                 K = Kill session"
  echo "                                 Q = Quit"
  echo ""
  echo "                           =============================="
  echo -e "${C2}                                           [Assignment 2]${RESET}"

  read -n1 -s KEY

  case "$KEY" in
    S|s)
      # Se sessione esiste: apri/riapri solo il terminale di attach
      if tmux has-session -t "$SESSION" 2>/dev/null; then
        kill_tmux_terminal
        open_tmux_terminal
      else
        # Crea la sessione in background (launcher non si blocca), poi attach in nuovo terminale
        kill_tmux_terminal
        start_tmux_session_detached
        sleep 0.4
        open_tmux_terminal
      fi
      ;;
    R|r)
      # Restart: kill session + terminal, ricrea detached, poi attach in nuovo terminale
      tmux kill-session -t "$SESSION" 2>/dev/null || true
      kill_tmux_terminal
      start_tmux_session_detached
      sleep 0.6
      open_tmux_terminal
      ;;
    K|k)
      tmux kill-session -t "$SESSION" 2>/dev/null || true
      kill_tmux_terminal
      ;;
    Q|q)
      clear
      exit 0
      ;;
  esac
done
