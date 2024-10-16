# This module should not be imported before the GUI is up otherwise, it'll be
# imported without `tr` and later import attempts will be ignored because this
# is own Python works.
# TODO: solve the import mess.

from __future__ import annotations

from itertools import islice
from itertools import zip_longest
import os
from pathlib import Path
import string
from typing import Any, Generator, Iterable, Optional
import xml.etree.ElementTree as et
from xml.dom import minidom
import yaml

import FreeCAD as fc

# Stubs and type hints.
DO = fc.DocumentObject
DOList = Iterable[DO]

# Otherwise et.tostring uses xlmns:ns0 as xacro namespace.
et.register_namespace('xacro', 'http://ros.org/wiki/xacro')


def add_path_to_environment_variable(path: [Path | str], env_var: str) -> None:
    """Add the path to the environment variable if existing.

    The environment variable is created if it does not exist.

    """
    path = Path(path).expanduser().absolute()
    if not path.exists():
        return
    path_str = str(path)
    if ' ' in path_str:
        path_str = f'"{path_str}"'
    if env_var not in os.environ:
        os.environ[env_var] = path_str
        return
    existing_paths = os.environ.get(env_var).split(':')
    if path_str not in existing_paths:
        os.environ[env_var] += ':' + path_str


def get_valid_filename(text: str) -> str:
    """Return a string that is a valid file name."""
    valids = string.ascii_letters + string.digits + '_-.'
    return ''.join(c if c in valids else '_' for c in text)


def warn_unsupported(
    objects: [DO | DOList],
    by: str = '',
    gui: bool = False,
) -> None:
    """Warn the user of an unsupported object type."""
    # Import here otherwise not fc.GuiUp.
    from .freecad_utils import warn

    if not isinstance(objects, list):
        objects = [objects]
    for o in objects:
        by_txt = f' by {by}' if by else ''
        try:
            label = o.Label
        except AttributeError:
            label = str(o)
        try:
            warn(
                f'Object "{label}" of type {o.TypeId}'
                f' not supported{by_txt}\n',
                gui=gui,
            )
        except AttributeError:
            warn(f'Object "{label}" not supported{by_txt}\n', gui=gui)


def attr_equals(instance: Any, attr: str, value: Any):
    return hasattr(instance, attr) and getattr(instance, attr) == value


def attr_is(instance: Any, attr: str, value: Any):
    return hasattr(instance, attr) and getattr(instance, attr) is value


def hasallattr(obj: Any, attrs: list[str]):
    """Return True if object has all attributes."""
    # Import here otherwise not fc.GuiUp.
    from .freecad_utils import warn

    if isinstance(attrs, str):
        # Developer help, call error.
        warn(f'hasallattr({attrs}) was replaced by hasallattr([{attrs}])')
        attrs = [attrs]
    for attr in attrs:
        if not hasattr(obj, attr):
            return False
    return True


def save_xml(
        xml: et.Element,
        filename: [Path | str],
) -> None:
    """Save the xml element into a file."""
    file_path = Path(filename)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    # Implementation note: we use minidom rather than
    # `et.ElementTree(xml).write(file_path, encoding='utf-8',
    # xml_declaration=True)` because the latter does not support pretty
    # printing.
    txt = minidom.parseString(et.tostring(xml)).toprettyxml(indent='  ')
    file_path.write_text(txt)


def save_yaml(
        content: str,
        filename: [Path | str],
        ) -> None:
    """Save the yaml content into a file."""
    with open(filename, 'w') as outfile:
        yaml.dump(content, outfile, default_flow_style=False, sort_keys=False)


def save_file(
        content: str,
        filename: [Path | str],
        ) -> None:
    """Save the content into a file."""
    file_path = Path(filename)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(content)


def grouper(iterable, n, fillvalue=None):
    """Collect data into fixed-length chunks or blocks."""
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"
    # From https://docs.python.org/3.8/library/itertools.html.
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)


def i_th_item(generator: Generator, i: int):
    """Return the i-th item of the generator."""
    return next(islice(generator, i, None))


def get_parent_by_pattern(
        file_path: [Path | str],
        pattern: str,
        type: Optional[str] = None,
) -> tuple[Path, str]:
    """Return the parent directory of the given file containing pattern.

    Return the directory that is parent (possibly indirect) of `filepath` and
    that contains the directory or file `pattern` as well as the file path
    relative to this parent path.

    If the file path is relative, return `(Path(), file_path)`.

    If the pattern was not found, return `(Path(), '')`.

    """
    file_path = Path(file_path).expanduser()
    if not file_path.is_absolute():
        return Path(), str(file_path)
    if type in ['f', 'file']:
        def is_correct_type(p: Path):
            return p.is_file()
    elif type in ['d', 'directory']:
        def is_correct_type(p: Path):
            return p.is_dir()
    else:
        def is_correct_type(p: Path):
            return p.exists()
    relative_file_path = ''
    while True:
        candidate_path_to_pattern = file_path / pattern
        if is_correct_type(candidate_path_to_pattern):
            return file_path, relative_file_path
        relative_file_path = (
            f'{file_path.name}/{relative_file_path}'
            if relative_file_path else file_path.name
        )
        file_path = file_path.parent
        if file_path.samefile(Path(file_path.root)):
            # We are at the root.
            return Path(), relative_file_path


def true_then_false(list: Iterable[bool]) -> bool:
    """Return True if no False is found after a True.

    >>> true_then_false([True])
    True
    >>> true_then_false([False])
    True
    >>> true_then_false([True, True, False])
    True
    >>> true_then_false([True, False, True])
    False
    >>> true_then_false([False, True, True])
    False

    """
    if not list:
        return True
    v0 = list[0]
    for v in list:
        if v0 or not v:
            v0 = v
            continue
        return False
    return True


def values_from_string(
    values_str: str,
    delimiters: Iterable[str] = (' ', ',', ';', '\t'),
) -> list[float]:
    """Return a list of floats from a string."""
    # Try a single value.
    try:
        return [float(values_str)]
    except ValueError:
        pass
    # Try a list of values separated by the given delimiters.
    for delimiter in delimiters:
        if delimiter in values_str:
            try:
                return [
                    float(value) for value in values_str.split(delimiter)
                    if value != ''
                ]
            except ValueError:
                pass
    return []
