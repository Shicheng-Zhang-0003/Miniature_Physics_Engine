# 472-MGB wire protocol (measured 2026-10-08 against s2tui at v9R4 head)

Transport: three pipes. Child stdin down (commands). Child stdout up
(command output; drained to a transcript file, informational only).
Child stderr up (banner, prompts, diagnostics) — the FRAMING channel.
This split is s2tui's own contract (display.h): stdout is DATA, stderr
is display, and the `s2>` prompt prints to stderr with fflush. An early
bridge revision framed on stdout and hung: the prompt never arrives
there. Completion signal is the next `s2>` prompt on stderr; the
snapshot frame itself travels via `sync` FILE, never stdout, so stdout
block-buffering can delay transcript text but never corrupt a frame.

## Handshake

On spawn s2tui prints a banner containing `s2tui` then the `s2>` prompt.
No banner identity, no session. EOF on stdin makes the child print `bye.`
and exit.

## Commands (down)

One shell line + `\n` per command; every command ends with a fresh `s2>`
prompt. Observed prompt forms: `s2> ` at buffer start and `\ns2> `.
`dd if=petri of=world` builds the 776-atom dish (prints a one-line world
summary). `ps` prints a SHORT status (one line + process list — NOT atom
data; the grid renders on TTY only). `sync <path>` writes the full
S2SAVE1 frame (this is the machine path).

## Frame grammar (S2SAVE1, up via file)

```
S2SAVE1
seed <ulong>
dt <f> cutoff <f> dielectric <f> temp <f> thermostat <0..3> tau <f> nu <f>
atoms <0..100000>
Z x y z q eps sigma vx vy vz          (%.17g, strtod-exact)
bonds <0..200000>
a b order r0 k                        (indices into atoms)
restraints <0..64>                    (5- or 6-col rows; counted in Phase 0)
<EOF>                                 (trailing bytes rejected)
```

All doubles must be finite; bond indices must satisfy `0 <= a,b < natoms`,
`a != b`. Anything else fails the whole frame.

## Timeouts / failure modes

Handshake 15 s, configure 30–60 s, snapshot 60 s (petri `sync` answers in
~1 s; budgets are generous, never infinite). Timeout, EOF without prompt,
missing/empty frame file, or parse failure marks the session dead:
further calls fail immediately, close kills (TERM grace, then KILL) and
reaps. The MPE side must freeze the last good frame with a banner — never
render fossils silently (copy-time TUI section `[s2bridge]`).

### Spawn failure is distinguishable

The wire contract above is reached only if the child started at all. Because
every pre-child failure used to return the same `NULL`, the bridge now
reports *which* one happened, with the OS errno where the OS has one
(`mgb_spawn_reason`):

| Status | Means |
|---|---|
| `mgb_spawn_e_no_path` | empty path |
| `mgb_spawn_e_notfound` | no such file (`ENOENT`) |
| `mgb_spawn_e_noexecbit` | exists but not executable (`EACCES`) |
| `mgb_spawn_e_noexec` | `exec` itself failed (`ENOEXEC`, …) |
| `mgb_spawn_e_pipe` / `_e_fork` / `_e_alloc` | resource failure before the fork completes |
| `mgb_spawn_e_handshake` | the child **did** start but never sent the banner/prompt (this is the only case that consumes the timeout) |

`exec` errno is carried back from the child through a `CLOEXEC` status pipe
read with `poll()`: on a successful `exec` the write end closes and the read
returns EOF immediately, on failure the child writes the real errno. Reading
that pipe non-blockingly instead **races** the child and reports every exec
failure as a handshake timeout.

Note the distinction the earlier single-`NULL` API could not express: a
reader must check the path before assuming a timeout, because "the file is
not executable" and "s2tui is wedged" have nothing in common operationally.
