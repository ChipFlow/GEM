# Handoff discipline

Handoffs in this project are **ephemeral working memory**, not historical record. They exist to bridge a single session boundary — when you stop working and someone else (Claude or human) picks up — and they are deleted once the work they describe is resolved.

This document defines what a handoff is, what it isn't, when to write one, and exactly what to do when one is resolved.

## Why this discipline exists

Decision rationale, technical context, and project state all have natural homes:

- **ADRs** (`docs/adr/`) capture architectural decisions and their *why*.
- **Design docs** (`docs/timing-model-extensions.md`, etc.) capture *how* things work.
- **Plan docs** (`docs/plans/phase-0-ir-and-oracle.md`, `post-phase-0-roadmap.md`) capture *what's left* and the next workstream slices.

When that content lives in a handoff instead, two things go wrong:

1. **It's not where contributors look.** A new contributor reading the README → SUMMARY → ADR chain shouldn't have to dig through a stack of resolved handoff docs to find load-bearing decisions or the current state of a workstream.
2. **It rots out of sync with reality.** Handoffs are point-in-time snapshots. A "STATUS: RESOLVED" banner doesn't help when the thing referenced has moved or changed; the canonical doc is what should hold the current truth.

The discipline closes this gap by **forcing migration before deletion**. Every load-bearing piece of a handoff lands in its proper home (ADR / design doc / plan doc) before the handoff file is removed.

## What a handoff IS

A handoff *lives* in its own dedicated directory, separate from the persistent plan docs whose content it eventually feeds: a single markdown file at `docs/handoffs/<topic>-handoff.md` containing exactly what the next session needs to pick up where you left off:

- **Goal & next-up** — what this session was trying to do, and what the *very next* concrete action is.
- **Done this session** — commits landed, with one-line summaries.
- **Open follow-ups** — the work that wasn't done, with enough scope detail to start cold.
- **Critical context** — gotchas, surprising findings, environment specifics that aren't obvious from the code or docs *yet*.
- **Verification** — the command(s) the next session runs to confirm the work is in the state you say it is.

Exactly one handoff exists at a time. There's no chain of resolved predecessors to wade through.

## What a handoff IS NOT

- **Not a decision log.** Decisions go in ADRs. If you find yourself writing "we chose X over Y because Z" in a handoff, that paragraph belongs in an ADR (or an existing ADR's "Consequences" / "Walk-back" section).
- **Not a design doc.** "How clock arrival flows from OpenSTA Tcl through the IR into the GPU constraint buffer" is a design topic; it lives in `docs/timing-model-extensions.md` Part B, not in a handoff's "Critical context" section.
- **Not a status dashboard for the project.** Workstream status lives in plan docs — `phase-0-ir-and-oracle.md` for current-phase WS state, `post-phase-0-roadmap.md` for forward-looking sequencing. A handoff cites those, doesn't reproduce them.
- **Not a historical record.** `git log` is the historical record. Handoffs that survive past their resolution turn into noise that misleads new contributors.

## When to write one

Write a handoff at the end of any session that:

1. **Leaves work in a partial state** that someone else might pick up cold.
2. **Captures non-obvious context** the next session needs (e.g. "the OpenSTA Tcl `find_timing` proc rejects `-full_update`; use `::sta::find_timing_cmd 1` directly").
3. **Documents the next concrete step** with enough scope to start without re-discovering it.

If the session ended at a clean stopping point (everything merged, all decisions documented in ADRs/plans, nothing surprising), don't write a handoff. The plan doc already says what's next.

## Resolution: fold, then delete

The two-location split is deliberate: handoffs *live* at `docs/handoffs/<topic>-handoff.md` while in flight; their *content* migrates into the persistent docs (`docs/adr/`, `docs/plans/`, design docs under `docs/`) at resolution. The handoff file then gets removed; nothing about the work is lost because everything load-bearing has a permanent home elsewhere.

When a handoff's work is done — whether in the next session or several sessions later — every load-bearing piece of it must be migrated to its proper home **before the handoff file is deleted**:

| If the handoff says... | It belongs in... |
|---|---|
| "We chose approach X over Y because Z" | The relevant ADR's Decision/Consequences section, or a new ADR if no fit exists |
| "Future scope for WS-N: do A then B then C" | The plan doc's WS-N section (`phase-0-ir-and-oracle.md` or successor) |
| "Gotcha: OpenSTA's Tcl X behaves Y" | A code comment near the Tcl call site, or a design doc if the gotcha cuts across files |
| "Build dep Z is required on Linux" | The build script's apt-suggestion / Brewfile / README install section |
| "Subsystem A doesn't yet do B" | Plan doc as a new open item, or an ADR-tracked walk-back if it's a deferred design choice |
| "Run `cargo test --feature foo` to verify" | The verification block in the relevant plan doc, or a test-running section in `CLAUDE.md` |

After migration, the handoff file is removed in the same commit as the migration:

```bash
git rm docs/handoffs/<topic>-handoff.md
git add <files-receiving-the-migrated-content>
git commit -m "$(cat <<'EOF'
docs: resolve <topic> handoff — fold into <where-it-went>

<one-paragraph summary of what was migrated and where>

Co-developed-by: Claude Code v<version> (<model-id>)
EOF
)"
```

The commit message records what migrated where — that's the audit trail. `git log -- docs/handoffs/` then shows the project's handoff history (one add, one delete per session) without needing the files themselves to live forever.

## Template

When you do need to write one, use this skeleton. Replace placeholders inline; delete sections that don't apply (better to omit a section than fill it with "N/A").

```markdown
# Handoff — <Topic> (one-line summary of what this session left open)

**Created:** YYYY-MM-DD
**Working tree:** clean | <state if not clean>
**Branch:** main | <branch>

## Goal & next-up

**Goal of this session:** <what you were trying to do, in 1–3 sentences>

**Next session should pick up:** <the very next concrete action, by name. Reference the plan doc section if applicable.>

**Verification command:**
```sh
<commands the next session runs to confirm this handoff's claimed state>
# Expect: <what success looks like>
```

## Done this session

| Commit | Subject | Notes |
|---|---|---|
| `<sha>` | <subject> | <one-line note> |

## Open follow-ups (priority-ordered)

### 1. <Item name> (<rough size>)

<Concrete scope. Enough detail to start cold. Link to existing plan/ADR/design-doc sections rather than reproducing them.>

### 2. ...

## Critical context

<Things the next session needs to know that aren't yet in the code/docs. Be honest about what's truly load-bearing — anything obvious from `git log` or a quick `grep` doesn't belong here.>

## References

- [`<predecessor-handoff if any>`](<path>) — predecessor (if relevant)
- [`<plan doc>`](<path>) — current workstream state
- [`<ADR>`](<path>) — relevant decision

---

**Resume in a new session with:**
\`\`\`
/resume_handoff docs/handoffs/<topic>-handoff.md
\`\`\`
```

## Tooling

The `create_handoff` and `resume_handoff` skills (from various Claude Code orchestration toolkits) generate and consume handoffs. They're optional — the discipline above is the load-bearing artifact. A handoff written by hand following this template is just as valid.

If you use one of those skills, expect it to default to YAML format under `thoughts/shared/handoffs/` with database indexing. **That doesn't apply to this project.** Override it: produce markdown at `docs/handoffs/<topic>-handoff.md` and skip the database step. The skill activation is informational; the project's convention takes precedence.
