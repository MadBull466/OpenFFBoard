# ODrive closeout policy attribution

2026-11-06. CDX-015 closeout addendum; this records existing O14/L11 choices
and changes no frames or peer behavior. It supersedes vendor-attribution wording
in protected charter/tests for closeout interpretation only.

The supplied ODrive 0.5.6 table/CSV and the independent complete-page review in
[FINAL_REVIEW_QC.md](reviews/FINAL_REVIEW_QC.md) establish 0x009 call/response RTR
semantics and the response layouts. The 0x017 entry lacks that call/response
asterisk; its Master footnote permits either axis and does not establish RTR.
Neither source specifies request DLC 0.

| Field or behavior | Attribution |
|---|---|
| Standard node/command ID encoding and response payload layout | Supplied vendor reference |
| RTR request and response for 0x009 | Supplied vendor call/response rule |
| Request DLC 0 for 0x009 and 0x017 | Adopted local O14/L11 policy; vendor fidelity unverified |
| RTR request and response trigger for 0x017 | Adopted local O14/L11 policy; vendor fidelity unverified |

The shared peer still strictly rejects a data request or nonzero request DLC for
both commands, and answers the adopted RTR requests. Its header now distinguishes
the supported and policy fields. These rejections establish local-policy
enforcement, not independent proof of hardware rejection. Ordinary exact-field
assertions in protected `test_odrive_frames.cpp` (O14/O42), and the peer strictness
tests, depend on that policy. A future approved policy change may require those
assertions to change without establishing a vendor nonconformance. No evidence
here establishes that existing firmware frames are wrong or that alternatives
work on a physical ODrive. Version/calibration differences remain unverified.

CDX-016 is a protected local title/body mismatch: the O14 case promising peer
answers checks request acceptance/counters. The P10 review's suppressed-response
experiment was detected by seven other cases (five peer reply/capture/delay
cases, O42 returned position and O38 voltage-request cadence), including direct
ID/payload observations. No duplicate response test was added solely for that
editorial mismatch. The protected title awaits its owner's edit.
