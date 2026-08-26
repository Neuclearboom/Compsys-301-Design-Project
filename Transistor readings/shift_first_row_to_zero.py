import csv
import sys
from pathlib import Path


def shift_first_row_to_zero(csv_path):
    path = Path(csv_path)

    with path.open("r", newline="") as f:
        rows = list(csv.reader(f))

    if not rows:
        raise ValueError("CSV is empty")

    first_value = float(rows[0][0])

    for row in rows:
        if not row:
            continue
        row[0] = f"{float(row[0]) - first_value:.5E}"

    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(rows)

    print(f"Updated {path} so the first row starts at 0.")


def append_csv(source_path, target_path):
    """Append source CSV to target CSV, adjusting timescale to remain increasing"""
    source = Path(source_path)
    target = Path(target_path)
    
    if not source.exists():
        raise ValueError(f"Source file {source} does not exist")
    if not target.exists():
        raise ValueError(f"Target file {target} does not exist")
    
    # Read source data
    with source.open("r", newline="") as f:
        source_rows = list(csv.reader(f))
    
    # Read target data
    with target.open("r", newline="") as f:
        target_rows = list(csv.reader(f))
    
    if not target_rows or not source_rows:
        raise ValueError("One or both files are empty")
    
    # Get last time from target and first time from source
    last_time = float(target_rows[-1][0])
    first_source_time = float(source_rows[0][0])
    
    # Calculate offset to continue timescale
    time_offset = last_time - first_source_time
    
    # Shift all source time values by the offset
    for row in source_rows:
        if row:  # Skip empty rows
            row[0] = f"{float(row[0]) + time_offset:.5E}"
    
    # Append shifted source rows to target rows
    target_rows.extend(source_rows)
    
    # Write back to target
    with target.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(target_rows)
    
    print(f"Appended {source} to {target}. Shifted timescale by {time_offset:.5E}. Total rows: {len(target_rows)}")


def reorganize_csv(csv_path, move_start=1, move_end=200, insert_at=2001):
    """
    Take rows from move_start to move_end and insert them after row insert_at-1
    Adjusts timescale to remain increasing.
    Rows are 1-indexed in the function parameters
    """
    path = Path(csv_path)
    
    with path.open("r", newline="") as f:
        rows = list(csv.reader(f))
    
    if not rows:
        raise ValueError("CSV is empty")
    
    # Convert to 0-indexed
    move_start_idx = move_start - 1
    move_end_idx = move_end
    insert_at_idx = insert_at - 1
    
    # Extract the rows to move (including the end index)
    rows_to_move = rows[move_start_idx:move_end_idx]
    
    # Create new row list without the moved rows
    remaining_rows = rows[:move_start_idx] + rows[move_end_idx:]
    
    # Get the timescale info
    first_moved_time = float(rows_to_move[0][0])
    time_before_insert = float(remaining_rows[min(insert_at_idx, len(remaining_rows)-1)][0])
    
    # Calculate offset to continue timescale
    time_offset = time_before_insert - first_moved_time
    
    # Shift all moved rows by the offset
    for row in rows_to_move:
        if row:  # Skip empty rows
            row[0] = f"{float(row[0]) + time_offset:.5E}"
    
    # Insert the moved rows at the new position
    new_rows = remaining_rows[:insert_at_idx] + rows_to_move + remaining_rows[insert_at_idx:]
    
    # Write back
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(new_rows)
    
    print(f"Reorganized {path}: moved rows {move_start}-{move_end} to position {insert_at}. Shifted timescale by {time_offset:.5E}. Total rows: {len(new_rows)}")


if __name__ == "__main__":
    if len(sys.argv) > 2:
        if sys.argv[1] == "reorganize":
            # reorganize <file> <move_start> <move_end> <insert_at>
            csv_file = sys.argv[2]
            move_start = int(sys.argv[3]) if len(sys.argv) > 3 else 1
            move_end = int(sys.argv[4]) if len(sys.argv) > 4 else 200
            insert_at = int(sys.argv[5]) if len(sys.argv) > 5 else 2001
            reorganize_csv(csv_file, move_start, move_end, insert_at)
        else:
            append_csv(sys.argv[1], sys.argv[2])
    else:
        target = sys.argv[1] if len(sys.argv) > 1 else "13l1l1.csv"
        shift_first_row_to_zero(target)
