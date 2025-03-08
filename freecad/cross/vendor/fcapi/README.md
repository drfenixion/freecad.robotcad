# FreeCAD Scripting Modern APIs

## Audience

This is for developers of FreeCAD extensions like Workbenches and Macros in pure
python.

This is a python API, it is expected that the readers are python developers with
basic python coding skills. The API does not use any obscure language feature but
it uses classes, functions, decorators, type hints, etc...

It is also expected that the readers are FreeCAD users, and have a good understanding
of the basic usage of it.

## Scripted Object API (Feature Python Object)

This API is an overlay API that helps in definition of Feature Python Objects in
a more declarative an pythonic way.

* `fpo.py `
  This is the API implementation
* `examples/`
  contains basic usage examples of the fpo api
* `docs/`
  contains source and generated documentation

### Documentation

* [docs/documentation.md](docs/documentation.md)
* [docs/documentation.pdf](docs/documentation.pdf)


## Declarative Qt/Gui API

This API provides a simple way to create GUIs for the extensions without the
complexity of raw Qt code or .ui files. Code layout reflect GUI structure making
it readable and easy to maintain. The API covers the most common Widgets, Qt is
a massive library so covering everything is virtually impossible for this project
and largely unnecessary.

* `fcui.py`
  This is the GUI API implementation
* `examples/ui/`
  contains basic usage examples of the `fcui` api
* `docs/`
  contains source and generated documentation


### Documentation
* [docs/ui-documentation.md](docs/ui-documentation.md)
* [docs/ui-documentation.pdf](docs/ui-documentation.pdf)


## Status

This is a working draft as of Nov 10, 2024