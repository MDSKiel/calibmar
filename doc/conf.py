import re
from pathlib import Path

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

version_string = None
with open(Path(__file__).parent.joinpath("../CMakeLists.txt"), "r") as f:
    version_string = re.search(
        r"project\(calibmar\s*VERSION\s(\S*)\s", f.read(), re.S
    ).group(1)
if version_string is None:
    raise "Could not determine version from CMakeLists.txt"

project = "Calibmar"
copyright = "2025, Felix Seegräber"
author = "Felix Seegräber"
version = version_string
release = version

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["myst_parser", "sphinx.ext.mathjax"]

myst_enable_extensions = [
    "dollarmath",
    "html_image",
    "smartquotes",
    "strikethrough",
    "tasklist",
    "linkify",
]

myst_heading_anchors = 3

templates_path = ["_templates"]
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_book_theme"
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]
html_context = {"default_mode": "light"}
html_title = "Calibmar"
html_theme_options = {
    "use_download_button": "false",
    "use_fullscreen_button": "false",
    "article_header_end": ["version", "theme-switcher"],
}
