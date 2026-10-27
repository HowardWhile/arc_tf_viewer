# Domain Docs

## Before exploring

- Read root `CONTEXT.md`, if present.
- Read relevant decisions under `docs/adr/`, if present.

If these do not exist, proceed silently. Do not flag their
absence or suggest creating them upfront. The domain-modeling
skill creates them lazily as terms and decisions are resolved.

## File structure

This repository uses a single-context layout:

```text
/
├── CONTEXT.md
├── docs/adr/
│   └── 0001-<decision-slug>.md
└── src/
```

## Use the glossary's vocabulary

Use domain terms as defined in `CONTEXT.md` when writing issue
titles, proposals, hypotheses, and test names. Avoid synonyms
the glossary explicitly rejects.

If a concept is missing, reconsider whether it belongs to
the project vocabulary; note genuine gaps for domain-modeling.

## Flag ADR conflicts

Explicitly identify any proposal that contradicts an existing
ADR, explaining why the decision should be reconsidered.
