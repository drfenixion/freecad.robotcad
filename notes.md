Initialization of objects
=========================

New object (using our `make_...` functions)
-------------------------------------------

- Object is created with `doc.addObject()`.
- ViewObject is created
- `Object.Proxy.__init__()`
- `ViewObject.Proxy.__init__()`
- `ViewObject.Proxy.attach()`, triggered by `ViewObject.Proxy = self`

When opening an existing document
---------------------------------

- Object exists
- ViewObject exists
- `ViewObject.Proxy.attach()`
- `Object.onDocumentRestored()`
