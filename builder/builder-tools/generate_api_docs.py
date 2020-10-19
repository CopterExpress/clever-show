import os
import sys
import pydoc
import logging


class DocsGenerator:
    def __init__(self,
                 module_header="# Module {}",
                 functions_header="## Module functions",
                 class_header="## Class {}",
                 function_header="### {}",
                 contents_table_header="# API documentation\n"
                                       "> Please note, following documentation files were automatically generated from the source code\n"
                                       "## Table of contents"
                 ):
        self.module_header = module_header
        self.functions_header = functions_header
        self.class_header = class_header
        self.function_header = function_header
        self.contents_table_header = contents_table_header

    def _get_full_markdown(self, module, links_path):
        output = [self.module_header.format(
            "[{}]({})".format(
                module.__name__,
                links_path
            ))
        ]

        if module.__doc__:
            output.append(module.__doc__)

        output.append(self.functions_header)
        output.extend(self._get_functions(module, links_path))
        output.extend(self._get_classes(module, links_path))

        return output

    def _get_classes(self, item, links_path):
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
            output.extend(self._get_functions(cls, links_path))  # Get the functions
            output.extend(self._get_classes(cls, links_path))  # Recurse into any subclasses
            output.append('\n')
        return output

    def _get_functions(self, item, links_path):
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

    @staticmethod
    def _write_lines(path, lines):
        with open(path, 'w') as f:
            for line in lines:
                f.write(line.replace("\n", os.linesep) + '\n')

    @staticmethod
    def _get_relpath(path1: str, path2: str) -> str:
        return os.path.sep.join(
            os.path.relpath(path1, path2).split(os.path.sep)[1:]
        )

    def generate_doc(self, module_import_path: str, output_path: str) -> None:
        try:
            module = pydoc.safeimport(module_import_path)

            if module is None:
                logging.error("Module not found")
                raise FileExistsError("Module not found")

        except pydoc.ErrorDuringImport as e:
            logging.error(f"Error while trying to import {module_import_path}: {e}")
            raise e
        else:
            relpath = self._get_relpath(module.__file__, output_path)
            docs = self._get_full_markdown(module, relpath.replace(os.path.sep, "/"))
            self._write_lines(output_path, docs)

    def generate_docs(self, modules: list, output_dir: str, contents_name="SUMMARY") -> None:
        contents_table = [self.contents_table_header]
        contents_path = os.path.realpath(os.path.join(output_dir, f"{contents_name}.md"))
        for module in modules:
            name = module[module.rfind('.') + 1::]
            path = os.path.realpath(os.path.join(output_dir, f"{name}.md"))
            self.generate_doc(module, path)

            relpath = os.path.sep.join(
                os.path.relpath(path, contents_path).split(os.path.sep)[1:]
            )
            contents_table.append(f"* [{name}]({relpath})")

        self._write_lines(contents_path, contents_table)


if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.realpath(__file__))
    sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir)))

    modules_list = [
        "lib.messaging",
        "drone.modules.client_core",
    ]

    gen = DocsGenerator(contents_table_header="# API documentation\n"
                                              "`clever-show` can be modified or used as set of modules to repurpose the software or implement different behaviors, expand functionality.\n"
                                              "> Please note, following documentation files were automatically generated from the source code\n"
                                              "## Table of contents")
    doc_path = os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir, "docs", "en", "api"))
    gen.generate_docs(modules_list, doc_path)
