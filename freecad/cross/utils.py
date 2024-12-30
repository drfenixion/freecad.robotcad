# This module should not be imported before the GUI is up otherwise, it'll be
# imported without `tr` and later import attempts will be ignored because this
# is own Python works.
# TODO: solve the import mess.

from __future__ import annotations

from itertools import islice
from itertools import zip_longest
from functools import reduce
import os
from pathlib import Path
from typing import Any, Generator, Iterable, Optional
import xml.etree.ElementTree as et
from xml.dom import minidom
import yaml
import collections.abc
import re
from copy import deepcopy
import hashlib

import xmltodict

import FreeCAD as fc

INVALID_FILENAME_CHARS = re.compile(r'[^\w.\-]+')

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
    path_sep = os.pathsep
    existing_paths = os.environ.get(env_var).split(path_sep)
    if path_str not in existing_paths:
        os.environ[env_var] += f'{path_sep}{path_str}'


def get_valid_filename(text: str) -> str:
    """Return a string that is a valid file name."""
    return INVALID_FILENAME_CHARS.sub('_', text)


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
    return getattr(instance, attr, None) == value


def attr_is(instance: Any, attr: str, value: Any):
    return getattr(instance, attr, None) is value


def hasallattr(obj: Any, attrs: list[str]):
    """Return True if object has all attributes."""
    # Import here otherwise not fc.GuiUp.
    from .freecad_utils import warn

    if isinstance(attrs, str):
        # Developer help, call error.
        warn(f'hasallattr({attrs}) was replaced by hasallattr([{attrs}])')
        attrs = [attrs]
    return all(hasattr(obj, attr) for attr in attrs)


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


def grouper(iterable: Iterable, n: int, fillvalue=None):
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


def true_then_false(booleans: Iterable[bool]) -> bool:
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
    return reduce(lambda a, b: (a[0] and a[1] >= b, b), booleans, (True, True))[0]


def values_from_string(
    values_str: str,
    delimiters: str = r'[ ,;\t]+',
) -> list[float]:
    """
    Return a list of floats from a delimited string.

    :param str values_str: delimited string
    :param str delimiters: regex pattern for valid delimiters, defaults to r'[ ,;\t]+'
    :return list[float]: a list containing ONLY the valid parsed floats
    """
    conversions = (str_to_float(v) for v in re.split(delimiters, values_str))
    return [v for v in conversions if v is not None]


def str_to_float(text: str, default: float | None = None) -> float | None:
    """
    Converts a string into a valid float number or returns a default value.

    :param str text: string representation of a float number
    :param float | None default: returned if conversion fails, defaults to None
    :return float | None: float number if conversion succeeds `default` if fails
    """
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def deepmerge(dict1, dict2):
    """ Merge dict like structures in deep nested levels """

    for k, v in dict2.items():
        if isinstance(v, collections.abc.Mapping):
            dict1[k] = deepmerge(dict1.get(k, {}), v)
        else:
            dict1[k] = v

    return dict1


def remove_key(dictionary: dict, key: str, recursively: bool = True):
    """
    Remove a key from a dictionary (recursively or not).

    :param dictionary: The dictionary from which to remove the key.
    :param key: The key to remove.
    :param recursively: Flag to remove key recursively.
    :return: The modified dictionary with the key removed.
    """
    if isinstance(dictionary, dict):
        # Iterate over a copy of the dictionary items to avoid runtime errors
        # when modifying the dictionary during iteration
        for k, v in list(dictionary.items()):
            if k == key:
                del dictionary[k]
            elif isinstance(v, dict):
                # Recursively call the function on nested dictionaries
                if recursively:
                    remove_key(v, key, recursively)
            elif isinstance(v, list):
                # If the value is a list, iterate over it to check for dictionaries
                if recursively:
                    for item in v:
                        if isinstance(item, dict):
                            remove_key(item, key, recursively)
    return dictionary


def dict_to_xml(
        dict: dict, keys_to_remove_before_convert: list = [], remove_keys_recursively: bool = True,
        full_document: bool = True, pretty: bool = False,
):
    """Convert dictionary to xml"""

    dict = deepcopy(dict)
    for key_to_remove in keys_to_remove_before_convert:
        dict = remove_key(dict, key_to_remove, remove_keys_recursively)

    xml_str = xmltodict.unparse(dict, full_document = full_document, pretty = pretty)
    return xml_str


def str_to_bool(s: str) -> bool:
    return s.lower() == "true"


def replace_substring_in_keys(dictionary, old_substring, new_substring):
    """
    Recursively replaces occurrences of a substring in dictionary keys.

    :param dictionary: The dictionary in which to perform the replacement.
    :param old_substring: The substring to be replaced.
    :param new_substring: The new substring to replace with.
    :return: A dictionary with replaced keys.
    """
    new_dict = {}
    for key, value in dictionary.items():
        # Replace the substring in the key
        new_key = key.replace(old_substring, new_substring)

        # If the value is a dictionary, recursively process it
        if isinstance(value, dict):
            new_dict[new_key] = replace_substring_in_keys(value, old_substring, new_substring)
        else:
            new_dict[new_key] = value

    return new_dict


def calc_md5(file_path: str) -> str:
    """
    Calculates the MD5 hash of a file's content.
    param:
        file_path: The path to the file.
    Return:
        The MD5 hash of the file's content.
    Raises:
        FileNotFoundError: If the file is not found.
        IOError: If an error occurs while reading the file.
    """
    md5 = hashlib.md5()
    with open(file_path, 'rb') as f:
        for chunk in iter(lambda: f.read(4096), b""):
            md5.update(chunk)
    return md5.hexdigest()