Create or update a milestone record for: $ARGUMENTS

Steps:
1. Read `docs/ROADMAP.md` and any existing `docs/milestones/M*.md` that this milestone depends on or relates to. Use grep first; read only the relevant parts.
2. Write `docs/milestones/M0X_MILESTONE_TITLE.md` (uppercase snake_case, number with no padding after `M`, e.g. `M01_AUTH.md`) using the template below. Keep every line short and direct.
3. Add or update its row in the table in `docs/ROADMAP.md` (`#`, milestone, state, link). Do not restate its content there.
4. Ask the user only for missing facts that cannot be inferred. Do not invent requirements.

Template:

```markdown
# M0X: Title

**State:** Planned | In progress | Done | Dropped
**Goal:** One sentence: the outcome.

## Relations
- Depends on: [M0Y](M0Y_TITLE.md) (or none)
- Enables: [M0Z](M0Z_TITLE.md) (or none)

## Requirements
- [ ] R1: Testable statement
- [ ] R2: ...

## Verification (built right)
- [ ] V1: Check, command or test, and expected result

## Validation (built the right thing)
- [ ] VA1: Who checks it, with what, and the success signal

## Log
- YYYY-MM-DD: What changed
```
