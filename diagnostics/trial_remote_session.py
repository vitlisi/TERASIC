"""Atomic per-trial folder creation, including the shared parent on first use."""
from pathlib import PurePosixPath
import shlex


def session_setup(remote,session_id):
    q=shlex.quote
    return ('mkdir -p '+q(str(PurePosixPath(remote).parent))+'; mkdir '+q(remote)+
            '; printf "%s\\n" '+q(session_id)+' > '+q(remote+'/session_id.txt')+'; ')
