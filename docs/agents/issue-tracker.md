# Issue tracker: Local Markdown

Issues and specs (PRDs) live as Markdown files in `.scratch/`.

## Conventions

- One feature per directory: `.scratch/<feature-slug>/`.
- Spec: `.scratch/<feature-slug>/spec.md`.
- One file per implementation ticket:
  `.scratch/<feature-slug>/issues/<NN>-<slug>.md`,
  numbered from `01`. Never combine all tickets into one file.
- Record triage state as a `Status:` line near the top of each
  issue file. Use the strings in `triage-labels.md`.
- Append comments and conversation history under `## Comments`.

## Publish to the issue tracker

Create the appropriate file under `.scratch/<feature-slug>/`,
creating directories as needed.

## Fetch the relevant ticket

Read the referenced file. The user normally provides its path
or issue number.

## Wayfinding operations

- Map: `.scratch/<effort>/map.md`, containing Notes,
  Decisions-so-far, and Fog.
- Child ticket: `.scratch/<effort>/issues/NN-<slug>.md`,
  numbered from `01`, with the question in its body.
- Record ticket type in `Type:`:
  `research`, `prototype`, `grilling`, or `task`.
- Wayfinding lifecycle uses `Status: claimed` or
  `Status: resolved`; these are separate from triage roles.
- Record dependencies near the top as `Blocked by: NN, NN`.
  A ticket is unblocked when all listed tickets are resolved.
- Frontier: scan for open, unblocked, unclaimed tickets;
  choose the lowest number.
- Claim: save `Status: claimed` before starting work.
- Resolve: append the answer under `## Answer`, set
  `Status: resolved`, and append a summary and ticket link
  to the map's Decisions-so-far.
