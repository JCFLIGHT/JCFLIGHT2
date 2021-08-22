import yaml
import codecs
import pathlib
import optparse
import os
import re


def Parse_Settings_YAML():
    with codecs.open("Settings.yaml", "r", "utf-8-sig") as Settings_YAML:
        return yaml.load(Settings_YAML, Loader=yaml.Loader)


def Generate_MD_Doc_From_YAML(Settings_YAML):
    Params = {}

    for Member in Settings_YAML['Members']:

        if not any(key in Member for key in ["Description", "DefaultValue", "Min", "Max"]) and not Options.Quiet:
            print("Setting \"{}\" tem especificação incompleta".format(
                Member['Name']))

        Params[Member['Name']] = {
            "Descrição": Member["Description"] if "Description" in Member else "",
            "Valor Padrão": Member["DefaultValue"] if "DefaultValue" in Member else "",
            "Min": Member["Min"] if "Min" in Member else "",
            "Max": Member["Max"] if "Max" in Member else ""
        }

    Output_Lines = []

    for Param in sorted(Params.items()):
        Output_Lines.extend([
            f"### {Param[0]}\n\n",
            f"{Param[1]['Descrição'] if Param[1]['Descrição'] else '_// Sem Descrição_'}\n\n",
            "| Valor Padrão | Min | Max |\n| --- | --- | --- |\n",
            f"| {Param[1]['Valor Padrão']} | {Param[1]['Min']} | {Param[1]['Max']} |\n\n",
            "---\n\n"
        ])

    return Output_Lines


def Regex_Search(Regex):
    with open(1, 'r') as File:
        for _, line in enumerate(File.readlines()):
            Matches = Regex.search(line)
            if Matches:
                yield Matches


def Find_dDefault(Setting_Name):
    Regex = re.compile(
        rf'^\s*\.{Setting_Name}\s=\s([A-Za-z0-9_\-]+)(?:,)?(?:\s+//.+$)?')
    Defaults = []
    for Matches in Regex_Search(Regex):
        Defaults.append(Matches.group(1))
    return Defaults


def Check_Defaults(settings_yaml):
    retval = True
    for Member in settings_yaml['Members']:

        Default_From_Code = Find_dDefault(Member['Name'])
        if len(Default_From_Code) == 0:
            continue
        elif len(Default_From_Code) > 1:
            if not Options.Quiet:
                print(
                    f"Valor duplicado de {Member['Name']}: {Default_From_Code}")
            continue

        Default_From_Code = Default_From_Code[0]

        Default_From_YAML = Member["DefaultValue"] if "DefaultValue" in Member else ""
        Default_From_YAML = Default_From_YAML.replace(
            '`', '').replace('*', '').replace('__', '')

        if Default_From_YAML not in Default_From_Code:
            if not Options.Quiet:
                print(
                    f"{Member['Name']} tem valores padrão incompatíveis. Relatórios de código '{Default_From_Code}' e relatórios YAML '{Default_From_YAML}'")
            retval = False
    return retval


try:
    print('')

except:
    print('')

with codecs.open(pathlib.PurePath('__main__').parent / 'Docs' / 'Settings.md', "w", "utf-8-sig") as Settings_md:
    global Options, Args
    parser = optparse.OptionParser()
    parser.add_option('-q', '--Quiet', action="store_true",
                      default=False, help="Não escreva nada")
    parser.add_option('-d', '--Defaults', action="store_true",
                      default=False, help="Verifique se há valores padrões incompatíveis")
    Options, Args = parser.parse_args()
    Settings_YAML = Parse_Settings_YAML()
    if Options.Defaults:
        Defaults_Match = Check_Defaults(Settings_YAML)
        quit(0 if Defaults_Match else 1)
    MD_Table = Generate_MD_Doc_From_YAML(Settings_YAML)
    Settings_md.writelines(["# Variáveis do CLI\n", "\n"] + MD_Table + ["\n",
                                                                        "> Esse arquivo é gerado automaticamente,não o edite manualmente!"])
