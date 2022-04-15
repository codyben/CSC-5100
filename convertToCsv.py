import csv, glob, os, json

for result in glob.glob("results/*/*/*.json"):
    new_file = result.replace("json", "csv")

    with open(result) as f:
        with open(new_file, "w+") as spreadsheet:

            parsed = json.load(f)
            header = tuple(list(list(parsed.values())[0].keys()) + ["id"])
            writer = csv.DictWriter(spreadsheet, header)
            writer.writeheader()
            
            for vid, data in parsed.items():
                data['id'] = vid
                writer.writerow(data)