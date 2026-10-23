"""The workspace root is found from a plain checkout and from a git worktree (whose .git is a file)."""
import os

from utility_objects import run_context


def _workspace(tmp_path, git_as_file):
    (tmp_path / 'src' / 'utility_objects' / 'utility_objects').mkdir(parents=True)
    if git_as_file:
        (tmp_path / '.git').write_text('gitdir: /elsewhere/.git/worktrees/x\n')
    else:
        (tmp_path / '.git').mkdir()
    return str(tmp_path / 'src' / 'utility_objects' / 'utility_objects' / 'run_context.py')


def test_root_found_in_a_worktree(tmp_path, monkeypatch):
    monkeypatch.setattr(run_context, '__file__', _workspace(tmp_path, git_as_file=True))
    assert run_context._detect_repo_root() == os.path.realpath(str(tmp_path))


def test_root_found_in_a_plain_checkout(tmp_path, monkeypatch):
    monkeypatch.setattr(run_context, '__file__', _workspace(tmp_path, git_as_file=False))
    assert run_context._detect_repo_root() == os.path.realpath(str(tmp_path))
