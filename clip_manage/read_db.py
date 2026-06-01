#!/usr/bin/env python3
"""
读取 clip_manage 生成的 SQLite 数据库。

用法:
  python3 read_db.py <db_path>                    # 打印摘要统计
  python3 read_db.py <db_path> --list              # 列出所有条目
  python3 read_db.py <db_path> --id 5              # 按 id 查询, 打印特征
  python3 read_db.py <db_path> --url <url>         # 按 url 查询
  python3 read_db.py <db_path> --id 5 --raw        # 按 id 查询, 输出原始 float 特征值
"""

import argparse
import sqlite3
import struct
import sys

FEATURE_DIM = 512  # CLIP 图像/文本特征维度


def decode_feature(blob: bytes) -> list[float]:
    """将 BLOB 解码为 float 列表."""
    num_floats = len(blob) // struct.calcsize("f")
    return list(struct.unpack(f"{num_floats}f", blob))


def print_item(row):
    """打印单条记录."""
    print(f"  id:        {row[0]}")
    print(f"  timestamp: {row[1]}")
    print(f"  type:      {'image' if row[2] else 'text'}")
    print(f"  name:      {row[3]}")
    print(f"  text:      {row[4]}")
    print(f"  url:       {row[5]}")
    feature = decode_feature(row[6])
    print(f"  feature:   [{feature[0]:.6f}, {feature[1]:.6f}, ..., {feature[-1]:.6f}]  (dim={len(feature)})")
    print(f"  extra:     {row[7]}")
    print()


def main():
    parser = argparse.ArgumentParser(description="读取 clip_manage SQLite 数据库")
    parser.add_argument("db_path", help="数据库文件路径, 如 clip.db")
    parser.add_argument("--list", action="store_true", help="列出所有条目")
    parser.add_argument("--id", type=int, help="按 id 查询")
    parser.add_argument("--url", type=str, help="按 url 查询")
    parser.add_argument("--raw", action="store_true", help="输出原始特征值 (配合 --id 使用)")
    args = parser.parse_args()

    conn = sqlite3.connect(args.db_path)
    cursor = conn.cursor()

    # 打印摘要
    cursor.execute("SELECT COUNT(*) FROM ClipItems")
    total = cursor.fetchone()[0]
    cursor.execute("SELECT COUNT(*) FROM ClipItems WHERE type = 1")
    image_count = cursor.fetchone()[0]
    cursor.execute("SELECT COUNT(*) FROM ClipItems WHERE type = 0")
    text_count = cursor.fetchone()[0]
    print(f"Database: {args.db_path}")
    print(f"  total: {total}  (image: {image_count}, text: {text_count})")
    print()

    # 按 id 查询
    if args.id is not None:
        cursor.execute(
            "SELECT id, timestamp, type, name, text, url, feature, extra "
            "FROM ClipItems WHERE id = ?", (args.id,)
        )
        row = cursor.fetchone()
        if row is None:
            print(f"No item found with id={args.id}")
        else:
            if args.raw:
                feature = decode_feature(row[6])
                for f in feature:
                    print(f)
            else:
                print_item(row)
        conn.close()
        return

    # 按 url 查询
    if args.url is not None:
        cursor.execute(
            "SELECT id, timestamp, type, name, text, url, feature, extra "
            "FROM ClipItems WHERE url = ?", (args.url,)
        )
        row = cursor.fetchone()
        if row is None:
            print(f"No item found with url={args.url}")
        else:
            print_item(row)
            feature = decode_feature(row[6])
            print("feature:", feature[:10])
            # for f in feature:
            #     print(f)
        conn.close()
        return

    # 列出所有条目
    if args.list:
        cursor.execute(
            "SELECT id, timestamp, type, name, text, url, feature, extra "
            "FROM ClipItems ORDER BY id"
        )
        rows = cursor.fetchall()
        for row in rows:
            print_item(row)
        print(f"--- {len(rows)} items total ---")

    conn.close()


if __name__ == "__main__":
    main()
