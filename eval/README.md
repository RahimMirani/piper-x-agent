# Evaluation

Fixed task prompts live in `tasks/`. They are version-controlled so every trial
across every model uses byte-identical wording; edit one and you have started a
new experiment, not continued the old one.

Keep the prompt free of strategy hints. The moment it suggests an approach, the
result measures the prompt.

Run trials with `scripts/run-trials.sh` and build the grading sheet with
`scripts/grade-trials.py`. The protocol is in [docs/eval-protocol.md](../docs/eval-protocol.md).
