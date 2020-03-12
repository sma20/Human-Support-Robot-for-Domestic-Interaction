#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import nltk
from nltk.corpus import stopwords
from nltk.corpus import wordnet
from std_msgs.msg import String

#rospy.init_node('speech_rec', anonymous=True)

audio = record = aup = None

action = [[ 'cup', 'bring'],[ 'milk', 'get'],[ 'water', 'get'],[ 'coke', 'get'],[ 'snacks', 'get'],[ 'tv-remote', 'get'],[ 'TV', 'turn on'],[ 'Tele Vision', 'turn on'],[ 'LED', 'turn on'],[ 'LCD', 'turn on'],[ 'sounds', 'turn on'],[ 'toaster', 'turn on'],[ 'microwave', 'turn on'],[ 'oven', 'turn on'],[ 'lights', 'turn on'],[ 'newspaper', 'get'], [ 'book', 'get'], [ 'glasses', 'get'], [ 'door', 'check'], [ 'location', 'tell'], [ 'door', 'get'],['location','go']]

area = [ 'bedroom', 'livingroom', 'bathroom','toilet','showerroom','restroom' 'sleepingroom', 'kitchen' ]
nouns = []
verbs = []
synonyms = []
task_comb = []


def talker(text):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('speech_rec', anonymous=True)
    pub.publish(text)

def createIntentNounPairs(intents, nouns):
    for x in nouns:
        for y in intents:
            task_comb.append([x,y])
    return task_comb

def unique_list(l):
    ulist = []
    [ulist.append(x) for x in l if x not in ulist]
    return ulist

def main():
    global audio, record, aup
    # obtain audio from the microphone
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = r.listen(source)
        #rospy.spin(1)
    speechToText = ""
    # recognize speech using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        speechToText = r.recognize_google(audio)
        #print("Google Speech Recognition thinks you said " + speechToText)
        return speechToText

    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")

    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


"""for row in rows:
    for i in nouns:
        if nouns[i] == row[2]:
            action = row['action']


if action.contains('BRING_MILK'):
    print ("call the method to bring the milk")"""

if __name__ == '__main__':
    #while not rospy.is_shutdown():

    text=main()


    if 'living' in text:
        text = text.replace('room','')
        text = text.replace('living', 'livingroom')

    print(text)

    """if 'shower' in text:
        text = text.replace('room','')
        text = text.replace('shower', 'showerroom')

    print(text)

    if 'rest' in text:
        text = text.replace('room','')
        text = text.replace('rest', 'restroom')

    print(text)

    if 'sleeping' in text:
        text = text.replace('room','')
        text = text.replace('sleeping', 'sleepingroom')

    print(text)"""

    #rospy.spin(10)

    if text != 'stop':
        sentence_re = r'(?:(?:[A-Z])(?:.[A-Z])+.?)|(?:\w+(?:-\w+)*)|(?:\$?\d+(?:.\d+)?%?)|(?:...|)(?:[][.,;"\'?():-_`])'
        lemmatizer = nltk.WordNetLemmatizer()
        stemmer = nltk.stem.porter.PorterStemmer()
        grammar = r"""
            NBAR:
                {<NN.*|JJ>*<NN.*>}  # Nouns and Adjectives, terminated with Nouns

            NP:
                {<NBAR>}
                {<NBAR><IN><NBAR>}  # Above, connected with in/of/etc...
        """
        chunker = nltk.RegexpParser(grammar)
        toks = nltk.regexp_tokenize(text, sentence_re)
        postoks = nltk.tag.pos_tag(toks)

        print(postoks)


        index_of_verbs = [i for i, v in enumerate(postoks) if v[1] == 'VB' or v[1] == 'VBG']


        for index in index_of_verbs:
            verbs.append(postoks[index])


        tree = chunker.parse(postoks)
        stopwords = stopwords.words('english')

        def leaves(tree):
            """Finds NP (nounphrase) leaf nodes of a chunk tree."""
            for subtree in tree.subtrees(filter = lambda t: t.label()=='NP'):
                yield subtree.leaves()

        def normalise(word):
            """Normalises words to lowercase and stems and lemmatizes it."""
            word = word.lower()
            # word = stemmer.stem_word(word) #if we consider stemmer then results comes with stemmed word, but in this case word will not match with comment
            word = lemmatizer.lemmatize(word)
            return word

        def acceptable_word(word):
            """Checks conditions for acceptable word: length, stopword. We can increase the length if we want to consider large phrase"""
            accepted = bool(2 <= len(word) <= 40
                and word.lower() not in stopwords)
            return accepted


        def get_terms(tree):
            for leaf in leaves(tree):
                term = [ normalise(w) for w,t in leaf if acceptable_word(w) ]
                yield term

        terms = get_terms(tree)


        for term in terms:
            for word in term:
                nouns.append(word)
                print(word)
            print

        print("verbs", verbs)
        print("nouns", nouns)
        #print("adj", Adjectives)

        i = 0
        for verb in verbs:
            print(verb[0])
            synonyms.append(verb[0])
            for syn in wordnet.synsets(verb[0]):
                for l in syn.lemmas():
                    synonyms.append(l.name())
            i =i+1

        intents = unique_list(synonyms)


        #print("intents_synonyms", intents)
        synonyms=[]
        j = 0
        for noun in nouns:
            print(noun)
            synonyms.append(noun)
            for syn in wordnet.synsets(noun):
                for l in syn.lemmas():
                    synonyms.append(l.name())
            j =i+1

        context = unique_list(synonyms)
        #print("nouns synonyms", context)

        #print("context_synonyms", context)
        #conn = sqliteDB.create_connection()
        #rows = sqliteDB.get_most_recent_entry(conn)
        #print(rows)
        room = ""
        for c in nouns:
            for a in area:
                if c == a:
                    room=a
        #print("room",room)
        #cartesian product of all possible combinations
        task_comb = createIntentNounPairs(intents, context)

        if len(nouns) == 0:
            task_comb = createIntentNounPairs(intents, intents)

        count = 0
        for tc in task_comb:
            for a in action:
                if tc[0] == a[0] and tc[1] == a[1]:
                    print("do the action " , tc, "to/from", room)
                    count = 1
                    t = "do the action ,"+  str(tc)+ "from, "+ str(room)
                    try:
                        talker(t)
                    except rospy.ROSInterruptException:
                        pass
                    break
                elif tc[1] == a[1]:

                    if tc[0] in area:
                        print("do the action " , tc, "to/from", tc[0])
                        t = "do the action ,"+  str(tc)+ "to/from, "+ str(tc[0])
                        count = 1
                        try:
                            talker(t)
                        except rospy.ROSInterruptException:
                            pass
                    break
            if count==1:
                break
