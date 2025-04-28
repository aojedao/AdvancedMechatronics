# Problem 1: Collect names and ages, then print names between 20 and 25 years old.
def collect_names_and_ages():
    n = int(input("Enter the number of entries: "))  # Step (i): Get number of entries
    people = []  # Initialize a list to store tuples of (name, age)

    for _ in range(n):  # Step (ii): Loop to get name and age pairs
        name = input("Enter name: ")  # Get name input
        age = input("Enter age: ")  # Get age input
        people.append((name, int(age)))  # Store as tuple in the list

    # Print names of people whose age is between 20 and 25
    print("People between 20 and 25 years old:")
    for name, age in people:
        if 20 <= age <= 25:  # Check age condition
            print(name)

# Call the function to execute
collect_names_and_ages()
