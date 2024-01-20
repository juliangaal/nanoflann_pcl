import csv
import xml.etree.ElementTree as ET
import argparse

header = ['Test Case', 'Flann Mean (ms)', 'Nanoflann Mean (ms)', 'Flann Std (ms)', 'Nanoflann Std (ms)']

def parse_xml(filename, categories):
    tree = ET.parse(filename)
    root = tree.getroot()
    data = []

    for testcase in root.findall('.//TestCase'):
        test_case_name = testcase.get('name')
        flann_mean = float(testcase.find(f'.//BenchmarkResults[@name="{categories[0]}"]/mean').get('value')) / 1e6
        flann_std = float(testcase.find(f'.//BenchmarkResults[@name="{categories[0]}"]/standardDeviation').get('value')) / 1e6
        nanoflann_mean = float(testcase.find(f'.//BenchmarkResults[@name="{categories[1]}"]/mean').get('value')) / 1e6
        nanoflann_std = float(testcase.find(f'.//BenchmarkResults[@name="{categories[1]}"]/standardDeviation').get('value')) / 1e6
        data.append([test_case_name, flann_mean, nanoflann_mean, flann_std, nanoflann_std])

    return data

def write_to_csv(data, csv_filename):
    with open(csv_filename, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)
        writer.writerows(data)

def format_float_if_float(value):
    try:
        float_value = float(value)
        return "{:.5f}".format(float_value)
    except ValueError:
        return value

def convert_to_markdown_table(data):
    markdown_table = "| " + " | ".join(header) + " |\n"
    markdown_table += "| " + " | ".join(["---"] * len(header)) + " |\n"

    for row in data:
        markdown_table += "| " + " | ".join(map(lambda x: format_float_if_float(x), row)) + " |\n"

    return markdown_table

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parse XML and convert to CSV and Markdown table.')
    parser.add_argument('xml_file', type=str, help='Path to the input XML file')
    parser.add_argument('--export_csv', action='store_true', help='Export data to CSV file')
    parser.add_argument('--categories', nargs='+', default=[], help='List of categories to filter data')

    args = parser.parse_args()

    assert(len(args.categories) == 2), "please provide two categories"
    assert(args.categories[0] != args.categories[1]), "categories must differ"
    assert('nanoflann' not in args.categories[0] and 'nanoflann' in args.categories[1]), "first category must be flann (to match header)"

    parsed_data = parse_xml(args.xml_file, args.categories)

    if args.export_csv:
        csv_filename = 'output.csv'
        write_to_csv(parsed_data, csv_filename)
        print(f'Data exported to CSV: {csv_filename}')

    markdown_table = convert_to_markdown_table(parsed_data)
    print(markdown_table)
