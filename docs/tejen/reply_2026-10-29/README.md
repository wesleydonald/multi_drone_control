# Tejen → Wesley: rig review, 29 October 2026

Start with `REPLY_TO_WESLEY.md`. It is suitable for Wesley or his coding agent.
`EVIDENCE_REQUEST.md` specifies a small return package; existing files are enough,
with no request to repeat flights or add logging before sending them.
`UPSTREAM_NOTES.md` identifies current source and one targeted existing commit.

Scope: review and evidence request only. No new code patch, launch override,
permission to change gates, or flight-readiness claim is included.

Basis:

- Wesley's `message_tejen_2026-10-29.md` and `CHANGES_FROM_TEJEN.md` as supplied.
- Tejen source revision `73e7b7fed86a584c7e6642dfc473c0fe9fe1a8a6`.
- Wesley's notes identify the latest upstream sync as `fd495db0`, with many
  subsequent local changes. His actual flown revision/profile needs confirming.
- The new rig logs have not been supplied or independently reconstructed here.
- Tejen additionally notes that stiff bends in the 1 m cable may create a
  persistent tip offset that is not oscillatory pendulum swing.

The review does not authorize a wholesale merge or transfer Tejen's standalone
obstacle-drone thrust calibration into the different payload controller.
