import os
import sys
import pydoc
import logging

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir)))


class DocsGenerator:
    def __init__(self,
                 links_raltive_path="",
                 module_header="# Module {}",
                 functions_header="## Module functions",
                 class_header="## Class {}",
                 function_header="### {}"
                 ):
        self.links_path = links_raltive_path
        self.module_header = module_header
        self.functions_header = functions_header
        self.class_header = class_header
        self.function_header = function_header

    def get_full_markdown(self, module, links_path):
        output = [self.module_header.format(
            "[{}]({})".format(
                module.__name__,
                links_path
            ))
        ]

        if module.__doc__:
            output.append(module.__doc__)

        output.append(self.functions_header)
        output.extend(self.get_functions(module, links_path))
        output.extend(self.get_classes(module, links_path))

        return output

    def get_classes(self, item, links_path):
        output = list()
        for cls_name, cls in pydoc.inspect.getmembers(item, pydoc.inspect.isclass):
            if cls_name.startswith("_") or cls_name == "__class__":
                continue
            if cls.__module__ != item.__name__:
                continue

            output.append(self.class_header.format(
                "[{}]({}#L{})".format(
                    cls_name,
                    links_path,
                    pydoc.inspect.getsourcelines(cls)[1]  # get source code line
                ))
            )
            output.append(pydoc.inspect.getdoc(cls) or '...')  # Get the docstring
            output.extend(self.get_functions(cls, links_path))  # Get the functions
            output.extend(self.get_classes(cls, links_path))  # Recurse into any subclasses
            output.append('\n')
        return output

    def get_functions(self, item, links_path):
        output = []
        for func_name, func in pydoc.inspect.getmembers(item, pydoc.inspect.isfunction):
            if func_name.startswith('_') and func_name != '__init__':
                continue
            # if func.__module__ != item.__name__:
            #     continue

            output.append(self.function_header.format(
                "[{}]({}#L{})".format(
                    func_name.replace('_', '\\_'),
                    links_path,
                    pydoc.inspect.getsourcelines(func)[1]  # get source code line
                ))
            )

            output.append('```py')
            output.append(f'def {func_name}{pydoc.inspect.signature(func)}')
            output.append('```')

            output.append(pydoc.inspect.getdoc(func) or "")  # get the docstring

        return output

    def generate_docs(self, module_import_path, output_path):
        try:
            module = pydoc.safeimport(module_import_path)

            if module is None:
                logging.error("Module not found")
                return
        except pydoc.ErrorDuringImport as e:
            logging.error(f"Error while trying to import {module_import_path}: {e}")
        else:
            relpath = os.path.sep.join(
                os.path.relpath(module.__file__, output_path).split(os.path.sep)[1:]
            )
            docs = self.get_full_markdown(module, relpath)
            with open(output_path, 'w') as f:
                for line in docs:
                    f.write(line.replace("\n", os.linesep) + '\n')


if __name__ == '__main__':
    modules_list = [
        "lib.messaging",
        "drone.modules.client_core",
    ]
    gen = DocsGenerator()
    for module in modules_list:
        name = module[module.rfind('.') + 1::]
        gen.generate_docs(module, os.path.realpath(
            os.path.join(current_dir, os.pardir, os.pardir, "docs", "en", "api", f"{name}.md")))
