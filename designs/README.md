# Bodies and commands

Each folder holds one design: `design.json` and `actions.npy`. Replay any of them with `python scripts/diagnose.py PATH/design.json`.

| Folder | What it is |
|---|---|
| `best-stock/` | The two-cell robot: a horizontal muscle on a rigid cell. 514.48 in the simulator as released. |
| `best-closed/` | The best design found with the loophole closed: a column of five vertical muscles. 13.83, and the same unpatched on x86-64 Linux; 58.52 unpatched in the shipped wheel on Apple silicon, where it is thrown too. A copy of `closed-search/base-15`. |
| `rule-variants/` | The best design found under each of the fifteen changed rule settings. Every one is a two-cell column. |
| `closed-search/` | The 35 designs from which the best with the loophole closed is chosen for each rule setting. |

`design.json`:

- `body`: rows of cells, top to bottom. 0 empty, 1 rigid, 2 soft, 3 horizontal muscle, 4 vertical muscle.
- `connections`: `null` joins every pair of neighbouring cells; otherwise two rows of cell indices, one join per column.
- `actions`: the file of commands, one row per command and one column per muscle, in the order the muscles appear reading the body row by row.
- `rules`: the rule setting it was found under.

Of the 35 in `closed-search/`, 31 were found by searches run under [the plain patch](../docs/evidence.md#the-plain-patch). The other four (`base-15`, `strain-02`, `dt-02`, `contact-02`) are the best of those for their setting, with the commands searched again with the loophole closed by [`scripts/retune.py`](../scripts/retune.py).

Every score is the best a search found, not a proved maximum.
