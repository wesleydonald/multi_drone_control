# Questions for Tejen before the combined M1 (send Fri 9 Oct; settle in person Wed 14 Oct)

What we need from him before R10a/R10b/R11, in the order it blocks us. His code keeps his values (lengths,
calibration); ours keeps ours (Wesley, 8 Oct).

| # | Question | Why we need it | Blocks |
|---|---|---|---|
| 1 | 11a0683f (measured-tip trim, `target_alignment_mode`): is 'legacy' still what he flies for M1, or should the combined runs use the measured-tip mode? | we port it with 'legacy' as the default | R10b |
| 2 | QUAD4 in his config: body 14, magnet tip 24, `/dev/QUAD4`, and his own values for it (rod length, tip offset, mass, thrust): which file, and are they current? | his nodes keep his values on QUAD4; ours keep kT the same as the other quads | R10a |
| 3 | The ball (pickup object): Motive body id (we assume 6) and its height on the floor | our mocap node becomes the only Motive client and republishes it for his nodes | R10a |
| 4 | Magnet gate: only one writer for QUAD4's magnet at a time (ours until his release, his after). Is that compatible with his pickup and drop commands? | two writers can drop the ring or the ball | R10b |
| 5 | His aux channels 5 and 7-10 zeroed on QUAD4 while our stack is up (we keep his magnet channel). Does anything of his need them in M1? | our radio node already carries them | R10b |
| 6 | A fault in his supervisor calls `land_now` (we land every drone on LAND; a fault lands, never disarms). Does `land_now` land QUAD4 from any phase of his mission, including carrying the ball? | Wesley's M1 rule: LAND lands every drone whatever controller has it | R10b |
| 7 | His supervisor in automatic for the combined runs: any manual step left in his M1 sequence (TAKEOFF accept, pickup confirm)? | the run sheet needs every keypress | R10b |
| 8 | Who reviews our edits to his code (logged in `src/tejen_mission/CHANGES_FROM_TEJEN.md`)? | we edit his fork, he should see every change | R10b |
| 9 | His laptop off the stack (no ROS on the network, same `ROS_DOMAIN_ID` not in use) during combined runs: OK? | everything runs on Wesley's laptop; a second ROS graph on the same domain duplicates topics | R10b |
| 10 | The 0.12 m marker offset bug in his tip marker (our M1 sim evidence was flown with it): is it fixed in his current tree, and does his rig config depend on it? | the weld height on our side | R10b |
