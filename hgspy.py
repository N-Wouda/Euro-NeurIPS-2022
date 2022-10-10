import glob
import importlib.machinery
import importlib.util
import sys


def get_module(where: str = "release/lib/hgspy*.so"):
    lib_path = next(glob.iglob(where))
    loader = importlib.machinery.ExtensionFileLoader("hgspy", lib_path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    hgspy = importlib.util.module_from_spec(spec)
    loader.exec_module(hgspy)

    return hgspy


sys.modules["hgspy"] = get_module()
