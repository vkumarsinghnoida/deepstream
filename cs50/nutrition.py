#!/usr/bin/python3

def nutr(frt):
    match frt:
        case 'apple':
            print("Calories: 130")
        case 'avocado':
            print("Calories: 50")
        case 'banana':
            print("Calories: 110")
        case 'Cantaloupe':
            print("Calories: 50")
        case 'grapefruit':
            print("Calories: 60")
        case 'grapes':
            print("Calories: 90")
        case 'honeydew melon':
            print("Calories: 50")
        case 'kiwifruit':
            print("Calories: 90")
        case 'lemon':
            print("Calories: 15")
        case 'lime':
            print("Calories: 20")
        case 'nectarine':
            print("Calories: 60")
        case 'orange':
            print("Calories: 80")
        case 'peach':
            print("Calories: 60")
        case 'pear':
            print("Calories: 100")
        case _:
            pass


def main():
    print(nutr(input("Item: ")))


if __name__ == "__main__":
    main()
