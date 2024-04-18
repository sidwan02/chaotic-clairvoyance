import pickle


def load_pickle(filename, verbose=False):
    if verbose:
        msgs = []
        try:
            afile = open(filename, "rb")
            while 1:
                try:
                    msg = pickle.load(afile)
                    msgs.append(msg)
                except EOFError:
                    break
            afile.close()
        except FileNotFoundError:
            print("Pickle file not found.")

        return msgs
    else:
        positions = []
        try:
            afile = open(filename, "rb")
            positions = pickle.load(afile)
            afile.close()
        except FileNotFoundError:
            print("Pickle file not found.")

        print(positions)
        return positions


if __name__ == "__main__":
    load_pickle()
