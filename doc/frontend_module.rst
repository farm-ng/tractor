.. _chapter-frontend_module:

Frontend
========

The frontend module provides a browser-based frontend as a GUI and graphical debugging tool.

The frontend participates on the eventbus via a WebRTC proxy, allowing it to introspect eventbus traffic and interact
with running services and programs.

It also exposes CRUD functionality for the blobstore, and a GUI for ``programd``.

The frontend aims to support an extensible approach to 2D and 3D visualization, such that
it is easy to build custom visualizations for application-specific data types, and
build full application-specific web apps.

TODO

- "end-user GUI"
- system diagram(s)
- Example of visualization
- Example of configuration
- Example of program
- How to extend it
