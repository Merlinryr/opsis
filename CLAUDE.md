# CLAUDE.md

## Save context
- Before reading a whole file, grep first and only read the parts you need.
- To understand the whole project, send a subagent to look and bring back only the summary.
- Show me the diff, not the whole file.
- Don't restate anything we've already decided in this session.
- Reuse existing logic across the app
- When we switch tasks, suggest /compact or a new session.
- Keep answers short by default.

## Coding rules
- Do no create unnecessary new abstractions
- Do not create implicit state machines.
- Always use coherent sources of truth.
- Always separate concerns.
- Always go for flexibility, modularity and simplicity when it comes to syntax. Keep things short, but accurate

## Documents architecture
You are to stick to the document architecture below:

- `docs/milestones/` holds the project milestones in the format `M0X_MILESTONE_TITLE.md`.
- `docs/research/` holds the technical research and brainstorming.
- `docs/reviews/` holds the architecture reviews.
