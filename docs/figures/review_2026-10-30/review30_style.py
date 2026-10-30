"""The 30 Oct review figures: the 23 Oct review look (../review_2026-10-23/review_style.py), reused, not copied, with
the audit's previews kept out of the repo (REVIEW_SCRATCH, default the system temp folder)."""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault('REVIEW_SCRATCH', os.path.join(tempfile.gettempdir(), 'review30_audit'))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'review_2026-10-23'))
import review_style as S  # noqa: E402,F401

S.HERE = HERE                 # save() writes beside this module, not into the 23 Oct folder
TEXT = '0.3'                  # labels in the data area: darker than S.GREY so they survive print (critic 30 Oct)
