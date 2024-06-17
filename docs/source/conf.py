# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import sys
from sphinx_needs.api.configuration import add_dynamic_function
from sphinx_needs.api.configuration import add_warning
sys.path.insert(0, os.path.abspath("../.."))


project = 'Autonomous Navigation'
copyright = '2024, Nedim Hosic'
author = 'Nedim Hosic'
release = '0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx_needs',
              'sphinx.ext.autodoc',
              'sphinx.ext.viewcode']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']


# === SPHINX NEEDS ===========================================================

needs_id_regex = r"^(REQ|IMPL|TEST)_[\d]{3,}"
needs_id_required = True
needs_collapse_details = True
# smartquotes = False

needs_types = [
    dict(
        directive="req",
        title="Requirement",
        prefix="REQ_",
        color="#BFD8D2",
        style="node",
    ),
    dict(
        directive="impl",
        title="Implementation",
        prefix="IMPL_",
        color="#DF744A",
        style="node",
    ),
    dict(
        directive="test",
        title="Test Case",
        prefix="TEST_",
        color="#DCB239",
        style="node",
    ),
    # Kept for backwards compatibility
    # dict(directive="need", title="Need", prefix="N_", color="#9856a5", style="node"),
]


# requirement -> implementation -> test
needs_extra_links = [
    {
        # requirement -> implementation
        "option": "implements",
        "incoming": "is implemented by",
        "outgoing": "implements",
        # "style": "#777777",
    },
    {
        # implementation -> test
        "option": "tests",
        "incoming": "is tested by",
        "outgoing": "tests",
        # "style": "#AA0000",
    },
]

needs_global_options = {
    # Apply dynamic style to all needs of "req" type
    "style": [
        ("[[status_based_style()]]", 'type=="req"'),
        ("blue_border, [[status_based_style()]]", 'type=="impl"'),
    ],
}


def need_dependencies_check(need, log):
    """Dynamic method for dynamic evaluation for "requirement" needs"""
    filters = {
        "req": ("implements_back", "Requirement %s has no implementation dependency"),
        "impl": ("tests_back", "Implementation %s has no test dependency"),
    }

    need_filter = filters.get(need.get("type"))
    if not need_filter:
        return
    if not need.get(need_filter[0]):
        log.warning(need_filter[1], need.get("id"))


def status_based_style(app, need, needs):
    """
    Evaluates the status of a requirement, and returns corresponding style
    """
    return {"open": "red_bar", "in progress": "yellow_bar", "done": "green_bar"}.get(
        need["status"], ""
    )


def setup(app):
    # here we register our custom/dynamic function
    add_dynamic_function(app, status_based_style)
    add_warning(app, "need_dependencies_check", need_dependencies_check)