# Replayable bodies and actions

These files let you reproduce a return without rerunning a search. `best-stock/` holds the two-cell horizontal-muscle-over-rigid design; `best-closed/` holds the best design found with the contact loophole closed. `rule-variants/` contains each changed rule's stock-contact winner. `closed-search/` contains the 31 candidates replayed to select the best admissible closed result for each rule.

Each directory pairs `design.json` with `actions.npy`:

- `body`: cell rows from top to bottom, normally 5×5, or 7×7 for `box7`. Values are 0 empty, 1 rigid, 2 soft, 3 horizontal muscle and 4 vertical muscle.
- `connections`: `null` joins every occupied side neighbour. An explicit array has two rows of cell indices, one join per column; indices follow the flattened body rows.
- `actions`: the action-file name, relative to the JSON file. The NumPy array is float64, with actions as rows and muscles as columns in occupied muscle-cell order through the flattened body. The base winner has shape `(500, 1)`.
- `rules`: the rule setting under which the design was found or scored.

Scores are best-found lower bounds, not optimality certificates. The closed searches were smaller. A candidate's folder records where it was found; it can win under another admissible rule. The checker handles action repetition for `control` and body embedding for `box7`. [Replay instructions](../REPRODUCE.md) cover both simulators and patched variants.
