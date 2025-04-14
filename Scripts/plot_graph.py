#!/usr/bin/env python3

import pandas
import matplotlib.pyplot as plt

df = pandas.read_csv('test_results.csv')

jpeg_df = df[df["compression_type"] == "jpeg"]
none_df = df[df["compression_type"] == "none"]

plt.figure(figsize=(12, 6))
bar_width = 0.35
index = range(len(jpeg_df))

plt.bar(
    index,
    jpeg_df["total_time"],
    bar_width,
    label="JPEG",
    color="blue"
)

plt.bar(
    [i + bar_width for i in index],
    none_df["total_time"],
    bar_width,
    label="None",
    color="red"
)

plt.xlabel("Image")
plt.ylabel("Total Time (s)")
plt.title("Total Time for JPEG vs None Compression")

plt.xticks(
    [i + bar_width / 2 for i in index],
    jpeg_df["image_path"],
    rotation=45,
    ha="right"
)

plt.legend()
plt.tight_layout()
plt.savefig("compression_time_comparison.png", dpi=300)
plt.show()


