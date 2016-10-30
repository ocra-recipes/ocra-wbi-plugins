---
layout: doxygen
title: Source Code & Documentation
permalink: /doxygen/
---

This page holds the Doxygen documentation for `ocra-recipes` and `ocra-wbi-plugins`.

The documentation is currently hosted in the `gh-pages` branch of the `ocra-wbi-plugins`. This is not meant to be something permanent, but it's so just for historic reasons. Moreover, the generation of the Doxygen documentation is a bit cumbersome. The procedure is as follows:

1. Create a directory called `html-gh-pages` in the `ocra-wbi-plugins` repository (don't worry, this will be ignored by git).
2. Checkout only the `gh-pages`.
3. Compile `ocra-recipes` and `ocra-wbi-plugins` with the CMake flag `GENERATE_DOCUMENTATION` active. Both projects will copy the generated documentation in the previously created directory.
4. Push your changes to `gh-pages` from within `html-gh-pages`
We need a way to automate this process and make it 'propre'.
