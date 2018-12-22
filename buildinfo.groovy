node('cv-basic') {

    println "Variables retrieved from handler job"
    println "gitBranch: " + gitBranch
    println "gitRefspec: " + gitRefspec
    println "gitRepo: " + gitRepo
    def packageVersion = version
    println "Version: " + version

    println "-----------------------------"

    def packageName = 'tra-path-finding'

    deleteDir()

    checkout([
        $class                           : 'GitSCM',
        branches                         : [[
            name: gitBranch
                                            ]],
        doGenerateSubmoduleConfigurations: true,
        extensions                       : [
        [$class: 'WipeWorkspace' ],
        [$class: 'SubmoduleOption', parentCredentials: true]
        ],

        userRemoteConfigs                : [[
            credentialsId: 'tra-jenkins-git',
            name         : 'origin',
            refspec      : gitRefspec,
            url          : gitRepo
                                            ]]
        ])



    sh '''
        apt-get install -y freeglut3-dev liburdfdom-dev liburdfdom-headers-dev libtbb2 libtbb-dev ruby ruby-dev rubygems build-essential
        gem install --no-ri --no-rdoc fpm
    '''

    if (packDeb=="true") {
        withEnv(['version=' + packageVersion]) {
            sh '''
              mkdir build
              cd build/
              cmake ..
              make -j 4
              make test
              cd ../deploy
              ./make-deb.sh
            '''
        }

        archiveArtifacts artifacts: 'project/*.deb', onlyIfSuccessful: true
        sshagent(credentials: ['apter-ssh-key']) {
            sh 'scp -o StrictHostKeyChecking=no project/*.deb apter@apt.tra.ai:/packages'
        }
    }

    sshagent(['tra-jenkins-git']){
                withCredentials([usernamePassword(credentialsId: 'tra-jenkins-gitlab-pass', passwordVariable: 'git_password', usernameVariable: 'git_username')]) {
                    sh '''
                        git config --global credential.helper cache
                        echo "machine git.tra.ai" > ~/.netrc
                        echo "login ${git_username}" >> ~/.netrc
                        echo "password ${git_password}" >> ~/.netrc
                        git submodule sync
                        git submodule update --buildFromFile --recursive
                    '''
                }
     }
}
