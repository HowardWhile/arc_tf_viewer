# Triage Labels

Map canonical triage roles to the strings used in issue
`Status:` lines.

| Role | Status string | Meaning |
| --- | --- | --- |
| needs-triage | needs-triage | Maintainer needs to evaluate |
| needs-info | needs-info | Waiting for reporter information |
| ready-for-agent | ready-for-agent | Fully specified for autonomous implementation |
| ready-for-human | ready-for-human | Requires human implementation |
| wontfix | wontfix | Will not be actioned |

When a skill asks to apply a triage label, update the issue's
`Status:` line using this mapping.

Edit the status strings here if the repository's vocabulary changes.
